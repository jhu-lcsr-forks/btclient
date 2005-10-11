
#define STK_SIZE 20
typedef struct {
  int prev_idx; //index of the object that allocated this one
  int idx; //index of this object
  void *obj;
}stkkey;

typedef struct stk_struct{
  struct stk_struct *next;

  size_t sizeofdata; //number of bytes in the stored object 
  size_t offsetofidx;
  size_t offsetofstk;
 // void *objs; //pointer to array of objects
  int max_objs;
  stkkey *keys; //array of free indexes (size: max_objs)
  int keys_end; //Number of indexes available
  int min_idx; //Tracks the amount we have depleted the stack.
}stk;

/**For a given array size, get a pointer to its stack
  if a stack for this size doesn't exist yet, create it*/
stk* get_stk_ptr(int objsize,int offset,int stkoffset); 
void* new_from_stk(stk* istk,int idx); //get a new vect_n from the stack
void release_from_stk(stk* istk,int idx);


typedef struct tst_struct
{
  int some;
  double dat;
  stk* stack;
  int stkidx;
  char name[32];
  struct tst_struct *parent,*next,**freelist;
}tst;


