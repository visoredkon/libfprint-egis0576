/*
 * Egis Technology Inc. (aka. LighTuning) 0576 driver for libfprint
 * Copyright (C) 2026 Marcel (Sprayxe) <sprayxe.marcel@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "egis0576"

#include "egis0576.h"
#include "drivers_api.h"


typedef enum {
  SEQ_INIT,
  SEQ_REPEAT,
  SEQ_POLL,
  SEQ_IMAGE
} seq_types;

typedef enum {
  DEV_OPEN,
  DEV_START,
  DEV_REQ,
  DEV_RESP,
  DEV_FULFILLED,
  NUM_STATES
} sm_states;

struct _FpDeviceEgis0576
{
  FpImageDevice parent;

  gboolean      running;
  gboolean      stop;
  gboolean      image_submitted;

  gboolean      has_background;
  guchar        background[EGIS0576_IMG_SIZE];

  seq_types     seq_type;
  int           seq_pkt_index;
  Egis0576Pkt   last_sent_pkt;
};
G_DECLARE_FINAL_TYPE (FpDeviceEgis0576, fpi_device_egis0576, FPI, DEVICE_EGIS0576, FpImageDevice);
G_DEFINE_TYPE (FpDeviceEgis0576, fpi_device_egis0576, FP_TYPE_IMAGE_DEVICE);


static void
upscale_and_pad_img (const guchar *src_img, guchar *canvas)
{
  const int src_w    = EGIS0576_IMG_WIDTH;
  const int src_h    = EGIS0576_IMG_HEIGHT;
  const int canvas_w = EGIS0576_CANVAS_WIDTH;
  const int offset_x = (EGIS0576_CANVAS_WIDTH  - EGIS0576_IMG_WIDTH_UPSCALE)  / 2;
  const int offset_y = (EGIS0576_CANVAS_HEIGHT - EGIS0576_IMG_HEIGHT_UPSCALE) / 2;

  memset (canvas, 255, EGIS0576_CANVAS_SIZE);

  for (gint sy = 0; sy < src_h; sy++)
    {
      const guchar *s0 = src_img + sy * src_w;
      const guchar *s1 = (sy + 1 < src_h) ? s0 + src_w : s0;
      guchar *d0 = canvas + ((sy * 2 + offset_y) * canvas_w + offset_x);
      guchar *d1 = d0 + canvas_w;

      for (gint sx = 0; sx < src_w - 1; sx++)
        {
          guchar tl = s0[0], tr = s0[1];
          guchar bl = s1[0], br = s1[1];
          s0++; s1++;

          d0[0] = tl;
          d0[1] = (guchar) ((tl + tr + 1) >> 1);
          d1[0] = (guchar) ((tl + bl + 1) >> 1);
          d1[1] = (guchar) ((tl + tr + bl + br + 2) >> 2);

          d0 += 2; d1 += 2;
        }

      guchar tl = s0[0], bl = s1[0];
      d0[0] = tl;
      d0[1] = tl;
      d1[0] = (guchar) ((tl + bl + 1) >> 1);
      d1[1] = d1[0];
    }
}

static void
normalize_img (const guchar *bg, guchar *img, gdouble *dark_portion)
{
  int min = 255;
  int max = 0;

  // Pass 1: find diff range (bg - img)
  for (gint i = 0; i < EGIS0576_IMG_SIZE; i++)
    {
      int d = (int) bg[i] - (int) img[i];
      if (d < min)
        min = d;
      if (d > max)
        max = d;
    }

  max -= EGIS0576_CONTRAST;
  min += EGIS0576_CONTRAST;
  int range = max - min;
  if (range <= 0)
    range = 1;

  // Pass 2: normalize using contrast range
  int count_ridges = 0;
  for (gint i = 0; i < EGIS0576_IMG_SIZE; i++)
    {
      int d = CLAMP ((int) bg[i] - (int) img[i], min, max);
      int normalized = 255 - ((d - min) * 255) / range;

      if (normalized < EGIS0576_RIDGE)
        {
          if (normalized < EGIS0576_CLAMP0)
            normalized = 0;
          count_ridges++;
        }
      else if (normalized > EGIS0576_CLAMP255)
        {
          normalized = 255;
        }

      img[i] = (guchar) normalized;
    }

  *dark_portion = (gdouble) count_ridges / EGIS0576_IMG_SIZE;
}

static void
capture_background (FpDevice *dev, FpiUsbTransfer *transfer, gdouble variance)
{
  FpImageDevice *img_self = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);
  guchar *img = transfer->buffer;

  if (variance < EGIS0576_BG_VARIANCE)
    {
      memcpy (self->background, img, EGIS0576_IMG_SIZE);
      self->has_background = TRUE;

      fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NEEDED);

      self->seq_type = SEQ_REPEAT;
      fpi_ssm_next_state_delayed (transfer->ssm, EGIS0576_DELAY_SHORT_MS);
      return;
    }

  // Variance above threshold, finger on sensor
  fpi_image_device_retry_scan (img_self, FP_DEVICE_RETRY_REMOVE_FINGER);

  self->seq_type = SEQ_REPEAT;
  fpi_ssm_next_state_delayed (transfer->ssm, EGIS0576_DELAY_LONG_MS);
}

static gboolean
handle_finger_quality (FpDevice *dev, FpiUsbTransfer *transfer, gboolean finger_present, gdouble dark_portion)
{
  FpImageDevice *img_self = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  if (finger_present && (dark_portion < EGIS0576_DARK_PORTION_MIN || dark_portion > EGIS0576_DARK_PORTION_MAX))
    {
      fpi_image_device_retry_scan (img_self, FP_DEVICE_RETRY_CENTER_FINGER);
      self->seq_type = SEQ_REPEAT;
      fpi_ssm_next_state_delayed (transfer->ssm, EGIS0576_DELAY_LONG_MS);
      return TRUE;
    }

  if (!finger_present)
    {
      self->seq_type = SEQ_REPEAT;
      fpi_image_device_report_finger_status (img_self, FALSE);
      fpi_ssm_next_state_delayed (transfer->ssm, EGIS0576_DELAY_SHORT_MS);
      return TRUE;
    }

  return FALSE;
}


static void
submit_finger_image (FpDevice *dev, FpiUsbTransfer *transfer)
{
  FpImageDevice *img_self = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);
  guchar *img = transfer->buffer;

  FpImage *fp_img = fp_image_new (EGIS0576_CANVAS_WIDTH, EGIS0576_CANVAS_HEIGHT);
  if (!fp_img)
    {
      fpi_ssm_mark_failed (transfer->ssm,
          fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL, "Failed to allocate image."));
      return;
    }

  upscale_and_pad_img (img, fp_img->data);

  fpi_image_device_report_finger_status (img_self, TRUE);
  fpi_image_device_image_captured (img_self, fp_img);

  self->image_submitted = TRUE;
  fpi_ssm_jump_to_state (transfer->ssm, DEV_START);
}

static void
process_finger (FpDevice *dev, FpiUsbTransfer *transfer)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);
  guchar *img = transfer->buffer;
  gdouble variance = fpi_std_sq_dev (img, EGIS0576_IMG_SIZE);

  if (!self->has_background)
    {
      capture_background (dev, transfer, variance);
      return;
    }

  gboolean finger_present = FALSE;
  gdouble dark_portion = -1;
  if (variance > EGIS0576_VARIANCE)
    {
      normalize_img (self->background, img, &dark_portion);
      finger_present = dark_portion > EGIS0576_DARK_PORTION;
    }

  fp_dbg ("Finger status (present, variance, dark port) : %d , %.2f, %.2f",
          finger_present, variance, dark_portion);

  if (handle_finger_quality (dev, transfer, finger_present, dark_portion))
    return;

  submit_finger_image (dev, transfer);
}

static void
process_poll_transfer (FpDevice *dev, FpiUsbTransfer *transfer)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  if (transfer->actual_length < EGIS0576_POLL_MIN_LEN)
    {
      fpi_ssm_mark_failed (transfer->ssm,
          fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID, "Device reported invalid poll."));
      return;
    }

  if ((transfer->buffer[EGIS0576_POLL_STATUS_IDX] & EGIS0576_POLL_READY_BIT) == EGIS0576_POLL_READY_BIT)
    {
      self->seq_type = SEQ_IMAGE;
      fpi_ssm_jump_to_state (transfer->ssm, DEV_REQ);
      return;
    }

  self->seq_pkt_index += 1;
  if (self->seq_pkt_index < EGIS0576_POLL_COUNT)
    {
      fpi_ssm_jump_to_state (transfer->ssm, DEV_REQ);
      return;
    }

  fpi_ssm_mark_failed (transfer->ssm,
      fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL, "Device exceeded maximum poll count."));
}

static void
process_image_transfer (FpDevice *dev, FpiUsbTransfer *transfer)
{
  guchar *buffer = transfer->buffer;
  gssize buffer_len = transfer->actual_length;

  if (buffer_len != EGIS0576_IMG_SIZE)
    {
      fpi_ssm_mark_failed (transfer->ssm,
          fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                    "Device image data size does not match expected size."));
      return;
    }

  // Heuristic: reject near-empty buffers
  guint sum = 0;
  for (gint i = 0; i < buffer_len; i++)
    sum += buffer[i];

  if (sum < (guint) buffer_len * EGIS0576_IMG_MIN_SUM_AVG)
    {
      fpi_ssm_mark_failed (transfer->ssm,
          fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID, "Device reported invalid data."));
      return;
    }

  process_finger (dev, transfer);
}


static void
cmd_resp_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  if (error)
    {
      fp_dbg ("During the %d sequence an error occurred at pkt index %d", self->seq_type,
              self->seq_pkt_index);
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  switch (self->seq_type)
    {
    // Init/Repeat response is echo, no data to process
    case SEQ_INIT:
    case SEQ_REPEAT:
      fpi_ssm_jump_to_state (transfer->ssm, DEV_REQ);
      break;

    case SEQ_POLL:
      process_poll_transfer (dev, transfer);
      break;

    case SEQ_IMAGE:
      process_image_transfer (dev, transfer);
      break;
    }
}

static void
recv_cmd_resp (FpiSsm *ssm, FpDevice *dev, const Egis0576Pkt *last_pkt)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EGIS0576_EPIN, last_pkt->res_len);

  transfer->ssm = ssm;

  fpi_usb_transfer_submit (transfer, EGIS0576_TIMEOUT, NULL, cmd_resp_cb, NULL);
}

static void
send_cmd_req (FpiSsm *ssm, FpDevice *dev, const Egis0576Pkt *pkt)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  self->last_sent_pkt = *pkt;
  fpi_usb_transfer_fill_bulk_full (transfer, EGIS0576_EPOUT, (guint8 *) pkt->cmd, pkt->len, NULL);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;

  fpi_usb_transfer_submit (transfer, EGIS0576_TIMEOUT, NULL, fpi_ssm_usb_transfer_cb, NULL);
}


static void
recv_cmd (FpiSsm *ssm, FpDevice *dev)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);
  const Egis0576Pkt *last_pkt = &self->last_sent_pkt;

  switch (self->seq_type)
    {
    case SEQ_INIT:
    case SEQ_REPEAT:
      {
      gboolean is_last_packet =
        (self->seq_type == SEQ_INIT   && self->seq_pkt_index == EGIS0576_INIT_PACKETS_LENGTH - 1) ||
        (self->seq_type == SEQ_REPEAT && self->seq_pkt_index == EGIS0576_REPEAT_PACKETS_LENGTH - 1);

      if (!is_last_packet)
        {
          recv_cmd_resp (ssm, dev, last_pkt);
          self->seq_pkt_index += 1;
        }
      else
        {
          self->seq_pkt_index = 0;
          self->seq_type = SEQ_POLL;
          fpi_ssm_jump_to_state (ssm, DEV_REQ);
        }
      }

      break;

    case SEQ_POLL:
    case SEQ_IMAGE:
      recv_cmd_resp (ssm, dev, last_pkt);
      break;
    }
}

static void
send_cmd (FpiSsm *ssm, FpDevice *dev)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  switch (self->seq_type)
    {
    case SEQ_INIT:
      g_assert (self->seq_pkt_index < EGIS0576_INIT_PACKETS_LENGTH);
      send_cmd_req (ssm, dev, &EGIS0576_INIT_PACKETS[self->seq_pkt_index]);
      break;

    case SEQ_REPEAT:
      g_assert (self->seq_pkt_index < EGIS0576_REPEAT_PACKETS_LENGTH);
      send_cmd_req (ssm, dev, &EGIS0576_REPEAT_PACKETS[self->seq_pkt_index]);
      break;

    case SEQ_POLL:
      send_cmd_req (ssm, dev, &EGIS0576_POLL_PACKET);
      break;

    case SEQ_IMAGE:
      send_cmd_req (ssm, dev, &EGIS0576_IMAGE_PACKET);
      break;
    }
}


static void
ssm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case DEV_OPEN:
      self->seq_type = SEQ_INIT;
      fpi_ssm_jump_to_state (ssm, DEV_START);
      break;

    case DEV_START:
      if (self->stop)
        {
          fp_dbg ("Deactivating device, marking completed.");
          fpi_ssm_mark_completed (ssm);
          return;
        }

      if (self->image_submitted)
        {
          fpi_ssm_next_state_delayed (ssm, EGIS0576_DELAY_MED_MS);
          return;
        }

      self->seq_pkt_index = 0;
      if (self->seq_type == SEQ_IMAGE)
        self->seq_type = SEQ_REPEAT;
      fpi_ssm_jump_to_state (ssm, DEV_REQ);
      break;

    case DEV_REQ:
      send_cmd (ssm, dev);
      break;

    case DEV_RESP:
      recv_cmd (ssm, dev);
      break;

    case DEV_FULFILLED:
      fpi_ssm_jump_to_state (ssm, DEV_START);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
sm_cb (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  self->running = FALSE;

  if (error && !self->stop)
    fpi_image_device_session_error (img_dev, error);
  else if (error)
    g_error_free (error);

  if (self->stop)
    {
      self->image_submitted = FALSE;
      fpi_image_device_deactivate_complete (img_dev, NULL);
    }
}


static void
dev_init (FpImageDevice *dev)
{
  GError *error = NULL;

  g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), EGIS0576_INTF, 0, &error);

  fpi_image_device_open_complete (dev, error);
}

static void
dev_deinit (FpImageDevice *dev)
{
  GError *error = NULL;

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), EGIS0576_INTF, 0,
                                  &error);

  fpi_image_device_close_complete (dev, error);
}

static void
dev_activate (FpImageDevice *dev)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);
  FpiSsm *ssm = fpi_ssm_new (FP_DEVICE (dev), ssm_run_state, NUM_STATES);

  self->image_submitted = FALSE;
  self->has_background = FALSE;
  self->stop = FALSE;
  self->running = TRUE;

  fpi_ssm_start (ssm, sm_cb);

  fpi_image_device_activate_complete (dev, NULL);
}

static void
dev_deactivate (FpImageDevice *dev)
{
  FpDeviceEgis0576 *self = FPI_DEVICE_EGIS0576 (dev);

  if (self->running)
    self->stop = TRUE;
  else
    fpi_image_device_deactivate_complete (dev, NULL);
}


static const FpIdEntry id_table[] = {
  {
    .vid = 0x1c7a,
    .pid = 0x0576,
  },
  {
    .vid = 0,
    .pid = 0,
  },
};

static void
fpi_device_egis0576_init (FpDeviceEgis0576 *self)
{
}

static void
fpi_device_egis0576_class_init (FpDeviceEgis0576Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = "egis0576";
  dev_class->full_name = "Egis Technology Inc. (aka. LighTuning) 0576";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = 40;
  dev_class->temp_hot_seconds = 0;

  img_class->img_open = dev_init;
  img_class->img_close = dev_deinit;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;

  img_class->img_width = EGIS0576_CANVAS_WIDTH;
  img_class->img_height = EGIS0576_CANVAS_HEIGHT;

  img_class->bz3_threshold = 9; /* security issue, can score more but not reliably */
}
