/*
 * Copyright (c) 2017, Xilinx Inc. and Contributors. All rights reserved.
 * Copyright (C) 2022-2023, Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*****************************************************************************
 * shmem_throughput_demo_task.c
 * This is the remote side of the shared memory throughput demo.
 * This demo does the following steps:
 *
 *  1. Get the shared memory device libmetal I/O region.
 *  1. Get the TTC timer device libemtal I/O region.
 *  2. Get IPI device libmetal I/O region and the IPI interrupt vector.
 *  3. Register IPI interrupt handler.
 *  6. Upload throughput measurement:
 *     Start TTC APU counter, write data to shared memory and kick IPI to
 *     notify remote. It will iterate for 1000 times, stop TTC APU counter.
 *     Wait for RPU IPI kick to know RPU has finished receiving packages
 *     and RPU TX counter is ready to read. Read the APU TX and RPU RX
 *     counter values and save them. Repeat for different package sizes.
 *     After this measurement, kick IPI to notify the remote, the
 *     measurement has finished.
 *  7. Download throughput measurement:
 *     Start TTC APU counter, wait for IPI kick, check if data is available,
 *     if yes, read as much data as possible from shared memory. It will
 *     iterates until 1000 packages have been received, stop TTC APU counter.
 *     Wait for RPU IPI kick so that APU can get the TTC RPU TX counter
 *     value. Kick IPI to notify the remote it has read the TTCi counter.
 *     Repeat for different package size.
 *  8. Cleanup resource:
 *     disable IPI interrupt and deregister the IPI interrupt handler.
 *
 * Here is the Shared memory structure of this demo:
 * |0x0   - 0x03         | number of APU to RPU buffers available to RPU |
 * |0x04  - 0x1FFFFF     | address array for shared buffers from APU to RPU |
 * |0x200000 - 0x200004  | number of RPU to APU buffers available to APU |
 * |0x200004 - 0x3FFFFF  | address array for shared buffers from RPU to APU |
 * |0x400000 - 0x7FFFFF  | APU to RPU buffers |
 * |0x800000 - 0xAFFFFF  | RPU to APU buffers |
 */

#include <unistd.h>
#include <metal/errno.h>
#include <metal/atomic.h>
#include <metal/io.h>
#include <metal/device.h>
#include <metal/alloc.h>
#include <metal/irq.h>
#include <metal/utilities.h>
#include "common.h"

#define TTC_CNT_APU_TO_RPU 2 /* APU to RPU TTC counter ID */
#define TTC_CNT_RPU_TO_APU 3 /* RPU to APU TTC counter ID */

#define TTC_CLK_FREQ_HZ	100000000
#define NS_PER_SEC 1000000000

/* Shared memory offsets */
#define SHM_DESC_OFFSET_TX 0x0
#define SHM_BUFF_OFFSET_TX 0x400000
#define SHM_DESC_OFFSET_RX 0x200000
#define SHM_BUFF_OFFSET_RX 0x800000

/* Shared memory descriptors offset */
#define SHM_DESC_AVAIL_OFFSET 0x00
#define SHM_DESC_ADDR_ARRAY_OFFSET 0x04

#define ITERATIONS 1000

#define NUM_ITER 512
#define BUF_SIZE_MAX 512
#define PKG_SIZE_MAX 1518
#define PKG_SIZE_MIN 16
#define TOTAL_DATA_SIZE (PKG_SIZE_MAX * NUM_ITER)

struct channel_s {
	struct metal_device *shm_dev; /* Shared memory metal device */
	struct metal_io_region *shm_io; /* Shared memory metal i/o region */
	struct metal_device *ttc_dev; /* TTC metal device */
	struct metal_io_region *ttc_io; /* TTC metal i/o region */
	atomic_flag remote_nkicked; /* 0 - kicked from remote */
};


#define RX_RING_SIZE (512)
#define TX_RING_SIZE (512)
#define ETH_PACKET_SIZE (1518)

struct packet {
  uint16_t size;
  uint8_t data[ETH_PACKET_SIZE];
};

struct ring {
  uint16_t head;
  uint16_t tail;
  uint16_t focus;
  struct packet buf[RX_RING_SIZE];
};

void init_ring(struct ring *ring)
{
	ring->head = 0;
	ring->tail = 0;
	ring->focus = 0;
}

void push_ring(struct ring *ring, uint8_t *buf, uint16_t size)
{
    uint16_t next_focus = (ring->focus + 1) % RX_RING_SIZE;
	memcpy(&ring->buf[ring->focus], buf, size);
	ring->focus = next_focus;
}

struct packet *pop_ring(struct ring *ring)
{
	struct packet *packet = &ring->buf[ring->tail];
	ring->tail += 1;
	ring->tail %= RX_RING_SIZE;
	return packet;
}

/**
 * @brief read_timer() - return TTC counter value
 *
 * @param[in] ttc_io - TTC timer i/o region
 * @param[in] cnt_id - counter ID
 */
static inline uint32_t read_timer(struct metal_io_region *ttc_io,
				unsigned long cnt_id)
{
	unsigned long offset = XTTCPS_CNT_VAL_OFFSET +
				XTTCPS_CNT_OFFSET(cnt_id);

	return metal_io_read32(ttc_io, offset);
}

/**
 * @brief reset_timer() - function to reset TTC counter
 *        Set the RST bit in the Count Control Reg.
 *
 * @param[in] ttc_io - TTC timer i/o region
 * @param[in] cnt_id - counter id
 */
static inline void reset_timer(struct metal_io_region *ttc_io,
				unsigned long cnt_id)
{
	uint32_t val;
	unsigned long offset = XTTCPS_CNT_CNTRL_OFFSET +
				XTTCPS_CNT_OFFSET(cnt_id);

	val = XTTCPS_CNT_CNTRL_RST_MASK;
	metal_io_write32(ttc_io, offset, val);
}

/**
 * @brief stop_timer() - function to stop TTC counter
 *        Set the disable bit in the Count Control Reg.
 *
 * @param[in] ttc_io - TTC timer i/o region
 * @param[in] cnt_id - counter id
 */
static inline void stop_timer(struct metal_io_region *ttc_io,
				unsigned long cnt_id)
{
	uint32_t val;
	unsigned long offset = XTTCPS_CNT_CNTRL_OFFSET +
				XTTCPS_CNT_OFFSET(cnt_id);

	val = XTTCPS_CNT_CNTRL_DIS_MASK;
	metal_io_write32(ttc_io, offset, val);
}

/**
 * @brief ipi_irq_handler() - IPI interrupt handler
 *        It will clear the notified flag to mark it's got an IPI interrupt.
 *        It will stop the RPU->APU timer and will clear the notified
 *        flag to mark it's got an IPI interrupt
 *
 * @param[in] vect_id - IPI interrupt vector ID
 * @param[in/out] priv - communication channel data for this application.
 *
 * @return - If the IPI interrupt is triggered by its remote, it returns
 *           METAL_IRQ_HANDLED. It returns METAL_IRQ_NOT_HANDLED, if it is
 *           not the interrupt it expected.
 *
 */
static int ipi_irq_handler (int vect_id, void *priv)
{
	struct channel_s *ch = (struct channel_s *)priv;

	(void)vect_id;

	if (ch) {
		atomic_flag_clear(&ch->remote_nkicked);
		return METAL_IRQ_HANDLED;
	}
	return METAL_IRQ_NOT_HANDLED;
}

/**
 * @brief measure_shmem_throughput() - Show throughput of using shared memory.
 *        - Upload throughput measurement:
 *          Start TTC APU counter, write data to shared memory and kick IPI to
 *          notify remote. It will iterate for 1000 times, stop TTC APU
 *          counter. Wait for RPU IPI kick to know RPU has finished receiving
 *          packages and RPU TX counter is ready to read. Read the APU TX and
 *          RPU RX counter values and save them. Repeat for different package
 *          sizes. After this measurement, kick IPI to notify the remote, the
 *          measurement has finished.
 *        - Download throughput measurement:
 *          Start TTC APU counter, wait for IPI kick, check if data is
 *          available, if yes, read as much data as possible from shared
 *          memory. It will iterates until 1000 packages have been received,
 *          stop TTC APU counter. Wait for RPU IPI kick so that APU can get
 *          the TTC RPU TX counter value. Kick IPI to notify the remote it
 *          has read the TTCi counter. Repeat for different package size.
 *
 * @param[in] ch - channel information, which contains the IPI i/o region,
 *                 shared memory i/o region and the ttc timer i/o region.
 * @return - 0 on success, error code if failure.
 */
static int measure_shmem_throughput(struct channel_s* ch)
{
	void *lbuf = NULL;
	int ret = 0;
	size_t i;
	uint32_t *apu_tx_count = NULL;
  	struct ring *rx_ring;
	struct ring *tx_ring;

	/* allocate memory for receiving data */
	lbuf = metal_allocate_memory(PKG_SIZE_MAX);
	if (!lbuf) {
		LPERROR("Failed to allocate memory.\r\n");
		return -ENOMEM;
	}
	memset(lbuf, 0xA, PKG_SIZE_MAX);

	/* allocate memory for saving counter values */
	apu_tx_count = metal_allocate_memory(sizeof(uint32_t));
	if (!apu_tx_count) {
		LPERROR("Failed to allocate memory.\r\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Clear shared memory */
	metal_io_block_set(ch->shm_io, 0, 0, metal_io_region_size(ch->shm_io));

	LPRINTF("Starting shared mem throughput demo\n");
	tx_ring = (struct ring *)metal_io_phys(ch->shm_io, 0);
	rx_ring = (struct ring *)metal_io_phys(ch->shm_io, sizeof(*tx_ring));

    reset_timer(ch->ttc_io, TTC_CNT_APU_TO_RPU);
    for (i = 0; i < NUM_ITER; i++) {
        push_ring(tx_ring, lbuf, PKG_SIZE_MAX);
        kick_ipi(NULL);
		wait_for_notified(&ch->remote_nkicked);
		if (rx_ring->head != rx_ring->focus) {
			rx_ring->head = rx_ring->focus;
		}

		while (rx_ring->tail != rx_ring->head) {
			struct packet *packet = pop_ring(rx_ring);
            (void)packet;
		}
	}

    stop_timer(ch->ttc_io, TTC_CNT_APU_TO_RPU);
    *apu_tx_count = read_timer(ch->ttc_io, TTC_CNT_APU_TO_RPU);

	/* Print the measurement result */
	float mbs = TTC_CLK_FREQ_HZ * (TOTAL_DATA_SIZE * 1.0 / (MB / 8));
    LPRINTF("    ping pong:    %u, %.1f Mb/s\n", *apu_tx_count,
			mbs / *apu_tx_count);
	LPRINTF("Finished shared memory throughput\n");

out:
	if (lbuf)
		metal_free_memory(lbuf);
	if (apu_tx_count)
		metal_free_memory(apu_tx_count);
	return ret;
}

int shmem_throughput_demo()
{
	struct metal_device *dev;
	struct metal_io_region *io;
	struct channel_s ch;
	int ret = 0;

	print_demo("shared memory throughput");
	memset(&ch, 0, sizeof(ch));

	/* Open shared memory device */
	ret = metal_device_open(BUS_NAME, SHM_DEV_NAME, &dev);
	if (ret) {
		LPERROR("Failed to open device %s.\n", SHM_DEV_NAME);
		goto out;
	}

	/* Get shared memory device IO region */
	io = metal_device_io_region(dev, 0);
	if (!io) {
		LPERROR("Failed to map io region for %s.\n", dev->name);
		ret = -ENODEV;
		goto out;
	}
	ch.shm_dev = dev;
	ch.shm_io = io;

	/* Open TTC device */
	ret = metal_device_open(BUS_NAME, TTC_DEV_NAME, &dev);
	if (ret) {
		LPERROR("Failed to open device %s.\n", TTC_DEV_NAME);
		goto out;
	}

	/* Get TTC IO region */
	io = metal_device_io_region(dev, 0);
	if (!io) {
		LPERROR("Failed to map io region for %s.\n", dev->name);
		ret = -ENODEV;
		goto out;
	}
	ch.ttc_dev = dev;
	ch.ttc_io = io;

	/* initialize remote_nkicked */
	ch.remote_nkicked = (atomic_flag)ATOMIC_FLAG_INIT;
	atomic_flag_test_and_set(&ch.remote_nkicked);

	ret = init_ipi();
	if (ret) {
		goto out;
	}
	ipi_kick_register_handler(ipi_irq_handler, &ch);
	enable_ipi_kick();

	/* Run atomic operation demo */
	ret = measure_shmem_throughput(&ch);

	/* disable IPI interrupt */
	disable_ipi_kick();
	deinit_ipi();

out:
	if (ch.ttc_dev)
		metal_device_close(ch.ttc_dev);
	if (ch.shm_dev)
		metal_device_close(ch.shm_dev);
	return ret;

}

