// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

//lock number
#define NLOCK 13
#define HASH(id) (id % NLOCK)

struct hash_lock {
  struct buf head;
  struct spinlock lock;
};

struct {
  //struct spinlock lock;
  struct buf buf[NBUF];

  // Linked list of all buffers, through prev/next.
  // Sorted by how recently the buffer was used.
  // head.next is most recent, head.prev is least.
  //struct buf head;

  struct hash_lock hash_l[NLOCK];
} bcache;

void
binit(void)
{
  struct buf *b;

  char name[10];
  for(int i = 0; i < NLOCK; i++)
  {
    snprintf(name, sizeof(name), "bcache_%d", i);
    initlock(&bcache.hash_l->lock, name);
    bcache.hash_l[i].head.prev = &bcache.hash_l[i].head;
    bcache.hash_l[i].head.next = &bcache.hash_l[i].head;
  }
  //initlock(&bcache.lock, "bcache");

  // Create linked list of buffers
  //bcache.head.prev = &bcache.head;
  //bcache.head.next = &bcache.head;
  for(b = bcache.buf; b < bcache.buf+NBUF; b++){
    b->next = bcache.hash_l[0].head.next;
    b->prev = &bcache.hash_l[0].head;
    initsleeplock(&b->lock, "buffer");
    bcache.hash_l[0].head.next->prev = b;
    bcache.hash_l[0].head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  acquire(&bcache.hash_l[HASH(blockno)].lock);

  // Is the block already cached?
  for(b = bcache.hash_l[HASH(blockno)].head.next; b != &bcache.hash_l[HASH(blockno)].head; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;

      acquire(&tickslock);
      b->timelast = ticks;
      release(&tickslock);

      release(&bcache.hash_l[HASH(blockno)].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached.
  // Recycle the least recently used (LRU) unused buffer.

  struct buf *temp;
  b = 0;
  int bid = HASH(blockno);
  for(int i = bid, cycle = 0; cycle != NLOCK; i = (i + 1) % NLOCK)
  {
    ++cycle;
    if(i != bid)
    {
      if(!holding(&bcache.hash_l[i].lock))
      {
        acquire(&bcache.hash_l[i].lock);
      }
      else
      {
        continue;
      }
    }

    for(temp = bcache.hash_l[i].head.next; temp != &bcache.hash_l[i].head; temp = temp->next)
    {
      if((temp->refcnt) == 0 && (b == 0 || temp->timelast < b->timelast))
      {
        b = temp;
      }
    }

    if(b)
    {
      if(i != bid)
      {
        b->next->prev = b->prev;
        b->prev->next = b->next;
        release(&bcache.hash_l[i].lock);
        b->next = bcache.hash_l[bid].head.next;
        b->prev = &bcache.hash_l[bid].head;
        bcache.hash_l[bid].head.next->prev = b;
        bcache.hash_l[bid].head.next = b;
      }
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      acquire(&tickslock);
      b->timelast = ticks;
      release(&tickslock);
      release(&bcache.hash_l[bid].lock);
      acquiresleep(&b->lock);
      return b;
    }
    else
    {
      if(i != bid)
      {
        release(&bcache.hash_l[i].lock);
      }
    }
  }
  /*
  for(b = bcache.hash_l[HASH(blockno)].head.prev; b != &bcache.hash_l[HASH(blockno)].head; b = b->prev){
    if(b->refcnt == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      release(&bcache.hash_l[HASH(blockno)].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
  */
  panic("bget: no buffers");
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  uint64 bid = HASH(b->blockno);
  acquire(&bcache.hash_l[bid].lock);
  b->refcnt--;
  acquire(&tickslock);
  b->timelast = ticks;
  release(&tickslock);
  /*
  b->refcnt--;
  if (b->refcnt == 0) {
    // no one is waiting for it.
    b->next->prev = b->prev;
    b->prev->next = b->next;

    b->next = bcache.head.next;
    b->prev = &bcache.head;
    bcache.head.next->prev = b;
    bcache.head.next = b;
  }
  */
  release(&bcache.hash_l[bid].lock);
}

void
bpin(struct buf *b) {
  acquire(&bcache.hash_l[HASH(b->blockno)].lock);
  b->refcnt++;
  release(&bcache.hash_l[HASH(b->blockno)].lock);
}

void
bunpin(struct buf *b) {
  acquire(&bcache.hash_l[HASH(b->blockno)].lock);
  b->refcnt--;
  release(&bcache.hash_l[HASH(b->blockno)].lock);
}


