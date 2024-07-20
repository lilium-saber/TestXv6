#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "riscv.h"
#include "spinlock.h"
#include "proc.h"
#include "defs.h"
#include "e1000_dev.h"
#include "net.h"

//在发送网络数据包时，首先会从tx_mbufs中取出一个数据包，然后在tx_ring中创建一个新的描述符，将数据包的元数据填入描述符，最后将描述符加入到环形缓冲区中。当网络设备准备好发送数据包时，就会根据tx_ring中的描述符来读取和发送tx_mbufs中的数据包。
#define TX_RING_SIZE 16

//接收描述符的环形缓冲区
static struct tx_desc tx_ring[TX_RING_SIZE] __attribute__((aligned(16)));

//存储数据包
static struct mbuf *tx_mbufs[TX_RING_SIZE];

#define RX_RING_SIZE 16
static struct rx_desc rx_ring[RX_RING_SIZE] __attribute__((aligned(16)));
static struct mbuf *rx_mbufs[RX_RING_SIZE];

// remember where the e1000's registers live.
// 指向e1000寄存器指针
static volatile uint32 *regs;

struct spinlock e1000_lock;

static struct spinlock tx_lock;
static struct spinlock rx_lock;

// called by pci_init().
// xregs is the memory address at which the
// e1000's registers are mapped.
void
e1000_init(uint32 *xregs)
{
  int i;

  initlock(&e1000_lock, "e1000");
  initlock(&tx_lock, "tx_lock");
  initlock(&rx_lock, "rx_lock");

  regs = xregs;

  // Reset the device
  // 设备复位
  regs[E1000_IMS] = 0; // disable interrupts
  regs[E1000_CTL] |= E1000_CTL_RST;
  regs[E1000_IMS] = 0; // redisable interrupts
  __sync_synchronize();

  // [E1000 14.5] Transmit initialization
  memset(tx_ring, 0, sizeof(tx_ring));
  for (i = 0; i < TX_RING_SIZE; i++) {
    tx_ring[i].status = E1000_TXD_STAT_DD;
    tx_mbufs[i] = 0;
  }
  regs[E1000_TDBAL] = (uint64) tx_ring;
  if(sizeof(tx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_TDLEN] = sizeof(tx_ring);
  regs[E1000_TDH] = regs[E1000_TDT] = 0;
  
  // [E1000 14.4] Receive initialization
  memset(rx_ring, 0, sizeof(rx_ring));
  for (i = 0; i < RX_RING_SIZE; i++) {
    rx_mbufs[i] = mbufalloc(0);
    if (!rx_mbufs[i])
      panic("e1000");
    rx_ring[i].addr = (uint64) rx_mbufs[i]->head;
  }
  regs[E1000_RDBAL] = (uint64) rx_ring;
  if(sizeof(rx_ring) % 128 != 0)
    panic("e1000");
  regs[E1000_RDH] = 0;
  regs[E1000_RDT] = RX_RING_SIZE - 1;
  regs[E1000_RDLEN] = sizeof(rx_ring);

  // filter by qemu's MAC address, 52:54:00:12:34:56
  regs[E1000_RA] = 0x12005452;
  regs[E1000_RA+1] = 0x5634 | (1<<31);
  // multicast table
  for (int i = 0; i < 4096/32; i++)
    regs[E1000_MTA + i] = 0;

  // transmitter control bits.
  regs[E1000_TCTL] = E1000_TCTL_EN |  // enable
    E1000_TCTL_PSP |                  // pad short packets
    (0x10 << E1000_TCTL_CT_SHIFT) |   // collision stuff
    (0x40 << E1000_TCTL_COLD_SHIFT);
  regs[E1000_TIPG] = 10 | (8<<10) | (6<<20); // inter-pkt gap

  // receiver control bits.
  regs[E1000_RCTL] = E1000_RCTL_EN | // enable receiver
    E1000_RCTL_BAM |                 // enable broadcast
    E1000_RCTL_SZ_2048 |             // 2048-byte rx buffers
    E1000_RCTL_SECRC;                // strip CRC
  
  // ask e1000 for receive interrupts.
  regs[E1000_RDTR] = 0; // interrupt after every received packet (no timer)
  regs[E1000_RADV] = 0; // interrupt after every packet (no timer)
  regs[E1000_IMS] = (1 << 7); // RXDW -- Receiver Descriptor Write Back
}

int e1000_transmit(struct mbuf *m)
{

  push_off();
  acquire(&tx_lock);
  int tx_index = regs[E1000_TDT];
  
  if(tx_index >= TX_RING_SIZE)
  {
    printf("tx_index out of bounds\n");
    release(&tx_lock);
    pop_off();
    return -1;
  }
  if(tx_ring[tx_index].status != E1000_TXD_STAT_DD)
  {
    printf("tx_ring[%d].status != E1000_TXD_STAT_DD\n", tx_index);
    release(&tx_lock);
    pop_off();
    return -1;
  }
  if(tx_mbufs[tx_index] != 0)
  {
    mbuffree(tx_mbufs[tx_index]);
    tx_mbufs[tx_index] = 0;
  }
  
  tx_mbufs[tx_index] = m;
  tx_ring[tx_index].status = E1000_TXD_STAT_DD;
  tx_ring[tx_index].addr = (uint64)m->head;
  tx_ring[tx_index].length = m->len;
  tx_ring[tx_index].cmd |= (E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);

  regs[E1000_TDT] = (regs[E1000_TDT] + 1) % TX_RING_SIZE;
  release(&tx_lock);
  pop_off();

  return 0;
}

static void e1000_recv(void)
{

  push_off();
  acquire(&rx_lock);
  int rx_index = (regs[E1000_RDT] + 1) % RX_RING_SIZE;

  if(rx_ring[rx_index].status == E1000_RXD_STAT_DD)
  {
    printf("rx_ring[%d].status == E1000_RXD_STAT_DD\n", rx_index);
    release(&rx_lock);
    pop_off();
    return;
  }
  if(rx_mbufs[rx_index] == 0)
  {
    printf("rx_mbufs[%d] == 0\n", rx_index);
    release(&rx_lock);
    pop_off();
    return;
  }
  
  rx_mbufs[rx_index]->len = rx_ring[rx_index].length;
  net_rx(rx_mbufs[rx_index]);

  rx_mbufs[rx_index] = mbufalloc(0);
  if(!rx_mbufs[rx_index])
  {
    panic("e1000 recv");
  }
  rx_ring[rx_index].addr = (uint64)rx_mbufs[rx_index]->head;
  rx_ring[rx_index].status = 0;

  regs[E1000_RDT] = rx_index;

  release(&rx_lock);
  pop_off();
}

void
e1000_intr(void)
{
  // tell the e1000 we've seen this interrupt;
  // without this the e1000 won't raise any
  // further interrupts.
  regs[E1000_ICR] = 0xffffffff;

  e1000_recv();
}
