/*======================================================================*
 *  Module .............libbt
 *  File ...............btmath.h
 *  Author .............Traveler Hauptman
 *  Creation Date ......18 Feb 2005
 *                                                                      *
 *  ******************************************************************  *
 *                                                                      *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                      *
 *======================================================================*/

#ifndef _BTMATH_H
#define _BTMATH_H

#ifdef __cplusplus
extern "C"
{
#endif/* __cplusplus */
 
#define PRACTICALLY_ZERO 0.0000001 //For avoiding divide by zero

#define mov_vn set_vn
//#define VECT_N_DEBUG
//#define VECT_N_BOUNDS  //perform bounds checking
#define VECT_N_MAXSIZE 100

//#define BTINLINE  
#define BTINLINE inline
#define MAX_VECTN_SIZE 100 //size of temporary doubles used for sscan_vn

typedef double btreal;

/** The list of all pointers allocated by btmath

  Btmemlist maintains a list of all pointers allocated by
  the btmath code and provides a function to free them all at once.
  This functionality is used internally. See addbtptr() for more 
  details. freebtptr() should be called at the end of every program
  that uses the btsystem library.
  
  
*/
typedef struct {
  int n,max;      //size of vector
  void **plist;  //pointer to data
}btmemlist;

void addbtptr(void *ptr);
void freebtptr();

/** An N element vector object.

vect_n is a linear algebra vector object. The elements are of type btreal. Function
definitions are in btmath.c. This 
object is optimized for speed and prefix notation. It occupies twice the memory
you would expect to accomplish this. In addition to standard vector math operations
there are some functions that will map an operation to the vector as an array of btreals 
similar to what matlab can do.

The vect_n object operator functions can be nested in prefix function form. set_vn() 
is the assignment function.

See new_vn(), and set_vn() for more information. A typical block of code using vect_n looks like:

\code 
      vect_n *a,*b; 
      
        //create a new object (required)
      a = new_vn(4);
      b = new_vn(4);
      
      const_vn(a,4.4,5.5,6.5,7.7);
      
      //b = a + 4 * a + a * a + a
      set_vn(b,add_vn(a,
               add_vn(scale_vn(4.0,a),
               add_vn(mul_vn(a,a),a))));
\endcode


\bug We should add a compiler define switched set of code to each function to
add in bounds checking and verbose error reporting

  Rules: The output of any function can be used as the input of another. The leftmost
  argument will hold the results;
  
  WARNING! There is no bounds checking in the code. All vect_n objects must be created 
  with new_vn() and if you are mixing vectors of different sizes be careful to read the
  btmath.c code so that you understand the dangers of doing so. 
*/

typedef struct barrett_vect_n{
  struct barrett_vect_n *ret;  //pointer to return data for stacked operations
  int n;      //size of vector
  btreal *q;  //pointer to data
}vect_n;

vect_n *new_vn(int size); //allocate an n-vector
void free_vn(vect_n *p);
int len_vn(vect_n *src); //number of elements in the vector
int sizeof_vn(vect_n *src); //return sizeof info for whole vector

int new_vn_group(int num, int size, ...); //allocate a group of n-vectors

void set_vn(vect_n* dest, vect_n* src); //assignment, copy
void setrange_vn(vect_n* dest, vect_n* src, int dest_start, int src_start, int num);
void extract_vn(btreal* dest, vect_n* src); //copy vector to a btreal array
void inject_vn(vect_n* dest, btreal* src); //copy btreal array to vector
btreal* valptr_vn(vect_n* src); //return a pointer to the btreal array holding the data
void setval_vn(vect_n* dest, int idx, btreal val); //function to set single value, zero indexed
btreal getval_vn(vect_n* dest, int idx);
vect_n* const_vn(vect_n* a, ...); //set vector to a constant array of btreals
void einit_vn(vect_n* dest,int i); // einit_vn(&a,3) = <0,0,0,1,0,0>
void fill_vn(vect_n* dest, btreal val);

vect_n* neg_vn(vect_n* a); //negate a vector
vect_n* scale_vn(btreal a, vect_n* v);
vect_n* add_vn(vect_n* a, vect_n* b);
vect_n* sub_vn(vect_n* a, vect_n* b);

//per element operation
vect_n* e_mul_vn(vect_n* a, vect_n* b); // Per Element multiply
vect_n* e_pow_vn(vect_n* a, btreal b);
vect_n* e_sqrt_vn(vect_n* a);
vect_n* e_sqr_vn(vect_n* a);

vect_n* wedge_vn(vect_n* a, vect_n*b); 
btreal  dot_vn(vect_n* a, vect_n* b);
btreal  norm_vn(vect_n* a); //euclidian norm
vect_n* unit_vn(vect_n* a); //unit vector
vect_n* interp_vn(vect_n* a, vect_n* b,btreal s); //linear interpolation, if s is greater than available, it is exstrapolated
vect_n* bound_vn(vect_n* a, btreal min, btreal max);
//int eq_vn(vect_n* a, vect_n* b); //equals
//int zero_vn(vect_n* a, vect_n* b); //zero vector test

void print_vn(vect_n* src);
char* sprint_vn(char *dest,vect_n* src);
char* sprint_cvs_vn(char *dest,vect_n* src);

int strcount_vn(char **src_string,char *delimiter); //Count the number of double values in a string
vect_n * sscan_vn(char *src);
vect_n * strto_vn(vect_n *dest,char *src,char *delimiters); //Convert a string to a vect_n
vect_n * cvsto_vn(vect_n* dest, char *src); //Convert a string into a vect_n
int test_vn(btreal error); //test & verification suite

/*===================== Vector Arrays=============================*/
/** A vector array object, each vector has the same number of elements.

vectray is an object for holding arrays of vectors of the same size. Function
definitions are in btmath.c.
A block of data is allocated all at once in the beginning. The vect_n data type 
is used to access it. All vectray objects must be created with new_vr().

\code
int test_vr(void)
{
  vectray *vr,*vr2;
  int cnt,idx;
  
  vr = new_vr(3,10);
  for (cnt = 0; cnt < 10; cnt++){
    fill_vn(idx_vr(vr,cnt),cnt);
    set_vn(idx_vr(vr,cnt),scale_vn(2.0,idx_vr(vr,cnt)));
    print_vn(idx_vr(vr,cnt));
  }
  
  read_csv_file_vr("test.csv",&vr2);
  for (cnt = 0; cnt < size_vr(vr2); cnt++){
    print_vn(idx_vr(vr2,cnt));
  }
}
\endcode

*/
typedef struct barrett_vectarray_n{
  btreal *data;
  vect_n *rayvect;
  int n;      //size of vector
  int rows,lastrow; //number of rows and present index
}vectray;

vectray * new_vr(int vect_size,int rows);
void destroy_vr(vectray *vr);

//Access
vect_n * mapdat_vr(vect_n *dest, vectray *ray, int idx); //map the data pointer of dest onto the index
BTINLINE vect_n * idx_vr(vectray *ray,int idx); // pointer into vectray
vect_n * getvn_vr(vect_n *dest,vectray *ray, int idx); //copy data at idx to dest
BTINLINE int endof_vr(vectray *ray); //returns the index of the last point
BTINLINE int sizeof_vr(vectray *ray);
BTINLINE int size_vr(vectray *ray); //ray->rows
//Add & Remove data
void append_vr(vectray *ray, vect_n* v);
void insertbefore_vr(vectray *ray, vect_n* v);
void insertafter_vr(vectray *ray, vect_n* v);
void remove_vr(vectray *ray, int idx);
void clear_vr(vectray *ray); //erase everything and set it to zero

//File I/O
int read_csv_file_vr(char *fileName, vectray **vr);

int test_vr(btreal error);
/*===================== Vector List=============================*/
/** this deals with arrays of vectors of the same size. A block of data 
is allocated dynamically. The vect_n data type is used to 
access it.

\bug This code is not yet implemented (and not yet necessary)
*/
typedef struct barrett_vectlist_n{
  vect_n *head,*tail;
  int n;      //max size of vector
  int rows;
}vectlist;

void init_vl(vectlist *vl);
void free_vl(vectlist *vl);
//Access

//Add & Remove data
void append_vl(vectlist *vl, vect_n* v);



/*=======================================================================*/
/** A vect_n object optimized for 3 elements

vect_3 is structurally compatible with vect_n. As such, they can be typecast
as each other and used in each others functions. set_v3() 
is the assignment function. See new_v3(), and set_v3() for more. Function
definitions are in btmath.c.

\code
vect_n *a;
vect_3 *b;

a = new_vn(5);
b = new_v3();

fill_v3((vect_3*)a,1.0); //= <1.0, 1.0, 1.0, 0.0, 0.0>
set_v3(b,(vect_3*)a); //b= <1.0, 1.0, 1.0>
set_v3(b,neg_vn((vect_n*)b)); //b= <-1.0, -1.0, -1.0>
\endcode
*/
typedef struct barrett_vect_3{
  struct barrett_vect_3* ret;  
  int n;      //size of vector
  btreal *q;
  btreal data[3];
}vect_3;

vect_3 * new_v3(); //allocate an n-vector
vect_3* set_v3(vect_3* dest, vect_3* src); //assignment, copy
void extract_v3(btreal* dest, vect_3* src); //copy vector to a btreal array
void inject_v3(vect_3* dest, btreal* src); //copy btreal array to vector
void setval_v3(vect_3* dest, int idx, btreal val); //function to set single value, zero indexed
btreal getval_v3(vect_3* dest, int idx);
vect_3* const_v3(vect_3* dest, btreal v1, btreal v2, btreal v3); //set vector to a constant array of btreals
vect_3* C_v3(btreal v1, btreal v2, btreal v3); //set vector to a constant array of btreals
void einit_v3(vect_3* dest,int i); // einit_vn(&a,3) = <0,0,0,1,0,0>
void fill_v3(vect_3* dest, btreal val);

vect_3* neg_v3(vect_3* a); //negate a vector
vect_3* scale_v3(btreal a, vect_3* v);
vect_3* add_v3(vect_3* a, vect_3* b);
vect_3* sub_v3(vect_3* a, vect_3* b);

vect_3* cross_v3(vect_3* a, vect_3*b); 
btreal  dot_v3(vect_3* a, vect_3* b);
btreal  norm_v3(vect_3* a); //euclidian norm
vect_3* unit_v3(vect_3* a); //unit vector

vect_3* bound_v3(vect_3* a, btreal min, btreal max);

char* sprint_v3(char *dest,vect_3* src);
void print_v3(vect_3* src);
//int eq_v3(vect_3* a, vect_3* b); //equals
//int zero_v3(vect_3* a, vect_3* b); //zero vector test

/*=======================================================================*/

/*=======================Quaternion===========================================*/
/** Quaternion object

The quat object is a quaternion; a 4 element imaginary number.
q = a + b*i + c*j + d*k 

The quaternion is stored in vector form as <a,b,c,d>. The quaternion objects use 
the same prefix function form as the vect_n object (and quat can be typecast as a
vect_n).

See new_q(), and set_q() for more. Function
definitions are in btmath.c.


*/
typedef struct barrett_quat{
  struct barrett_quat* ret;  
  int n;      //size of vector
  btreal *q;
  btreal data[4];
}quat;

quat * new_q(); //allocate an n-vector
quat* set_q(quat* dest, quat* src); //assignment, copy
void extract_q(btreal* dest, quat* src); //copy vector to a btreal array
void inject_q(quat* dest, btreal* src); //copy btreal array to vector
void setval_q(quat* dest, int idx, btreal val); //function to set single value, zero indexed
btreal getval_q(quat* dest, int idx);
quat* const_q(quat* dest, btreal v1, btreal v2, btreal v3, btreal v4); //set vector to a constant array of btreals
quat* C_q(btreal v1, btreal v2, btreal v3, btreal v4); //set vector to a constant array of btreals
void fill_q(quat* dest, btreal val);


quat* conj_q(quat* a); //!< Conjugate. =s + -1*v
quat* inv_q(quat* a); //!< Inverse
quat* exp_q(quat* a); //!< Exponential
quat* log_q(quat* a); //!< Log o
quat* neg_q(quat* a); //negate a vector
quat* scale_q(btreal a, quat* v);
quat* add_q(quat* a, quat* b);
quat* sub_q(quat* a, quat* b);

quat* mul_q(quat* a, quat*b); 
quat* pow_q(quat* a, btreal b);
btreal dot_q(quat* a, quat* b);
btreal norm_q(quat* a); //euclidian norm
quat* unit_q(quat* a); //unit vector


btreal GCdist_q(quat* start, quat* end); //!< Great circle distance between two quaternions
btreal angle_q(quat* src); //!< Returns the angle represented by the quaternion
vect_3* axis_q(vect_3* dest, quat* src); //!< Returns the unit vector representing the rotation
vect_3* GCaxis_q(vect_3* dest, quat* start, quat* end); //!< Great circle axis between two quaternions

quat* slerp_q(quat* q0,quat* q1,btreal t);

char* sprint_q(char *dest,quat* src);
void print_q(quat* src);
//int eq_v3(vect_3* a, vect_3* b); //equals
//int zero_v3(vect_3* a, vect_3* b); //zero vector test

/*=======================================================================*/
/** homogeneous 4x4 matrix optimized for robots. 4th row is not computed or stored.
*/
typedef struct barrett_matr_h{
  struct barrett_matr_h *ret;
  btreal *q;
  btreal data[16];
  int m,n; //m rows, n cols
}matr_h;

matr_h * new_mh();
BTINLINE void set_mh(matr_h* dest, matr_h* src);
BTINLINE void ident_mh(matr_h* dest); // identity matrix
BTINLINE void setrow_mh(matr_h* dest,int row, btreal s1, btreal s2, btreal s3, btreal s4);
void getcol_mh(vect_3* dest, matr_h* src, int n); //get the specified column
//void getpos_mh(vec_3* dest, matr_h* src); //get the last column
BTINLINE btreal getval_mh(matr_h* src, int row, int col);
//void setval_mh(matr_h* src, int row, int col, btreal val);
//void setrow_mh(matr_h* src, int row, btreal v1, btreal v2, btreal v3, btreal v4 );
//void getrot_mh(matr_n* dest, matr_h* src);
BTINLINE matr_h* mul_mh(matr_h* a,matr_h* b);
BTINLINE void setmul_mh(matr_h* a,matr_h* b); //multiply and store in a
BTINLINE vect_3* matXvec_mh(matr_h* a, vect_3* b);
void print_mh(matr_h* src);
int test_mh(btreal error);
/*=======================================================================*/
/** 3x3 rotation matrix

Function definitions are in btmath.c.
*/
typedef struct barrett_matr_h matr_3;

matr_3 * new_m3(); //allocates memory and sets to identity
BTINLINE void set_m3(matr_3* dest, matr_3* src);
BTINLINE void setrow_m3(matr_h* dest,int row, btreal s1, btreal s2, btreal s3);

BTINLINE void ident_m3(matr_3* dest); // identity matrix

//void zero_m3(matr_3* dest); // zero matrix
BTINLINE void getcol_m3(vect_3* dest, matr_3* src, int n); //get the specified column
BTINLINE btreal getval_m3(matr_3* src, int row, int col);

BTINLINE matr_3* mul_m3(matr_3* a,matr_3* b);
BTINLINE void setmul_m3(matr_3* a,matr_h* b); //multiply and store in a
BTINLINE matr_3* T_m3(matr_3* a); //transpose
BTINLINE vect_3* matXvec_m3(matr_3* a, vect_3* b);
BTINLINE vect_3* matTXvec_m3(matr_3* a, vect_3* b); //matXvec_m3(T_m3(a),b);

void print_m3(matr_3* src);
int test_m3(btreal error);
BTINLINE vect_3* RtoXYZf_m3(matr_3* R, vect_3* XYZ); //return ZYZ
BTINLINE matr_3* XYZftoR_m3(matr_3* R, vect_3* XYZ); //Return R
BTINLINE vect_3* eqaxis_m3(matr_3* R, vect_3* a); //return axis of rotation with amount encoded in length
quat* R_to_q(quat* dest, matr_3* src);
matr_3* q_to_R(matr_3* dest, quat* src);
//vect_3* EUL_m3(matr_3* src); //return Euler angles
//vect_3* RPY_m3(matr_3* src); //return Roll pitch yaw
/*=======================================================================*/
/** general matrix.

Function definitions are in btmath.c.
*/
typedef struct barrett_matr_mn{
  struct barrett_matr_mn *ret;
  btreal *q;
  int m,n; //m rows, n cols
}matr_mn;

matr_mn * new_mn();
BTINLINE void set_mn(matr_mn* dest, matr_mn* src);
BTINLINE void ident_mn(matr_mn* dest); // identity matrix
BTINLINE void setrow_mn(matr_mn* dest,int row, vect_n* src);
BTINLINE void setcol_mn(matr_mn* dest,int row, vect_n* src);
void getcol_mn(vect_n* dest, matr_mn* src, int n); //get the specified column

BTINLINE btreal getval_mn(matr_mn* src, int row, int col);
BTINLINE matr_mn* mul_mn(matr_mn* a,matr_mn* b);
BTINLINE void setmul_mn(matr_mn* a,matr_mn* b); //multiply and store in a
BTINLINE vect_n* matXvec_mn(matr_mn* a, vect_n* b);
void print_mn(matr_mn* src);
int test_mn(btreal error);

/*=======================================================================*/
/** A digital filter object

btfilter is a digital filter object. new_btfilter() allocates memory for a new object.
init_btfilter_*() will create simple generic filters. eval_btfilter() is used for the
actual filter calculation. See new_btfilter() for more. Function definitions are in btmath.c.

*/
typedef struct
{
  int   order;             // FIRST_ORDER, SECOND_ORDER, OR FOURTH_ORDER
  int size;
  
  btreal  zeta;        // damping factor (between 0 and 1)
  
  int   index;         // index of current value in input and output arrays
  btreal  *d;  // coefficients of denominator
  btreal  *n;  // coefficients of numerator
  btreal  *x;  // old input values
  btreal  *y;  // old output values
    
} btfilter;
btfilter * new_btfilter(int size);
btreal  eval_btfilter(btfilter *filt, btreal xnew);
void  init_btfilter_diff(btfilter *filt, int order, btreal sample_time, btreal cutoffHz);
void  init_btfilter_butterworth_diff(btfilter *filt, btreal sample_time, btreal cutoffHz);
void  init_btfilter_lowpass(btfilter *filt, btreal sample_time, btreal cutoffHz);


/* Define the Filter structure */
typedef struct
{
  int   order;             // FIRST_ORDER, SECOND_ORDER, OR FOURTH_ORDER
  int size;
  
  btreal  zeta;        // damping factor (between 0 and 1)
  
  int   index;         // index of current value in input and output arrays
  vect_n  *d;  // coefficients of denominator
  vect_n  *n;  // coefficients of numerator
  vect_n  *x;  // old input values
  vect_n  *y;  // old output values
    
} btfilter_vn;
btfilter_vn * new_btfilter_vn(int size,int vsize);
vect_n * eval_btfilter_vn(btfilter_vn *filt, vect_n *xnew);
void  init_btfilter_vn_diff(btfilter_vn *filt, int order, btreal sample_time, btreal cutoffHz);





BTINLINE btreal atan2_bt(btreal arg1, btreal arg2); 
BTINLINE btreal interp_bt(btreal x1, btreal y1, btreal x2, btreal y2, btreal x); //linear interpolation



#ifdef __cplusplus
}
#endif/* __cplusplus */
#endif /*_BTMATH_H */

/*======================================================================*
 *                                                                      *
 *          Copyright (c) 2005 Barrett Technology, Inc.           *
 *                        625 Mount Auburn St                           *
 *                    Cambridge, MA  02138,  USA                        *
 *                                                                      *
 *                        All rights reserved.                          *
 *                                                                      *
 *  ******************************************************************  *
 *                            DISCLAIMER                                *
 *                                                                      *
 *  This software and related documentation are provided to you on      *
 *  an as is basis and without warranty of any kind.  No warranties,    *
 *  express or implied, including, without limitation, any warranties   *
 *  of merchantability or fitness for a particular purpose are being    *
 *  provided by Barrett Technology, Inc.  In no event shall Barrett     *
 *  Technology, Inc. be liable for any lost development expenses, lost  *
 *  lost profits, or any incidental, special, or consequential damage.  *
 *======================================================================*/
