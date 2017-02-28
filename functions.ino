#include <SPI.h>
#include <Wire.h>
const int MPUADDR=0x68;
struct MouseInfo {
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;

  uint16_t voltage = 0;
  uint8_t buttons = 0;
} my_data;
typedef enum : uint8_t {
  GYRO_PREC_250 = 0,
  GYRO_PREC_500,
  GYRO_PREC_1000,
  GYRO_PREC_2000
} gyro_precision_e;

typedef enum : uint8_t {
  ACCEL_PREC_2 = 0,
  ACCEL_PREC_4,
  ACCEL_PREC_8,
  ACCEL_PREC_16
} accel_precision_e;
void setSleep(bool enable)
{
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x6B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUADDR,1,true);
  uint8_t power=Wire.read();
  if(enable){
    power = (power|0b01000000);
  }
  else{
    power = (power&0b10111111);
  }
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x6B);
  Wire.write(power);
  Wire.endTransmission(true);
  Serial.println("sleep");
}

void getAccelData( int16_t* ax,int16_t* ay, int16_t* az)
{
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUADDR,6,true);
  *ax=Wire.read() << 8|Wire.read();
  *ay=Wire.read() << 8|Wire.read();
  *az=Wire.read() << 8|Wire.read();
  Wire.endTransmission(true);
  
}
void getGyroData( int16_t* gx,int16_t* gy, int16_t* gz)
{
  Wire.beginTransmission(MPUADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPUADDR,6,true);
  *gx=Wire.read() << 8|Wire.read();
  *gy=Wire.read() << 8|Wire.read();
  *gz=Wire.read() << 8|Wire.read();
  Wire.endTransmission(true);
}

void setGyroPrec(uint8_t prec) {
  Wire.beginTransmission(MPUADDR);
  prec &= 0b11;
  prec=prec<<3;
  Wire.write(0x1B);
  //Wire.endTransmission(false);
  Wire.write(prec);
  Wire.endTransmission(true);
  //Serial.println("setG"); 
}

void setAccelPrec(uint8_t prec)
{
  Wire.beginTransmission(MPUADDR);
  prec&=0b11;
  prec=prec<<3;
  Wire.write(0x1C);
  Wire.write(prec);
  Wire.endTransmission(true);
  Serial.println("setA");
}


int Matrix_Inverse(double* A, int n)
{
    // A = input matrix AND result matrix
    // n = number of rows = number of columns in A (n x n)
    int pivrow;     // keeps track of current pivot row
    int k,i,j;      // k: overall index along diagonal; i: row index; j: col index
    int pivrows[n]; // keeps track of rows swaps to undo at end
    double tmp;      // used for finding max value and making column swaps
 
    for (k = 0; k < n; k++)
    {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < n; i++)
        {
            if (abs(A[i*n+k]) >= tmp)   // 'Avoid using other functions inside abs()?'
            {
                tmp = abs(A[i*n+k]);
                pivrow = i;
            }
        }
 
        // check for singular matrix
        if (A[pivrow*n+k] == 0.0f)
        {
            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }
 
        // Execute pivot (row swap) if needed
        if (pivrow != k)
        {
            // swap row k with pivrow
            for (j = 0; j < n; j++)
            {
                tmp = A[k*n+j];
                A[k*n+j] = A[pivrow*n+j];
                A[pivrow*n+j] = tmp;
            }
        }
        pivrows[k] = pivrow;    // record row swap (even if no swap happened)
 
        tmp = 1.0f/A[k*n+k];    // invert pivot element
        A[k*n+k] = 1.0f;        // This element of input matrix becomes result matrix
 
        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < n; j++)
        {
            A[k*n+j] = A[k*n+j]*tmp;
        }
 
        // Now eliminate all other entries in this column
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                tmp = A[i*n+k];
                A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
                for (j = 0; j < n; j++)
                {
                    A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
                }
            }
        }
    }
 
    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = n-1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                tmp = A[i*n+k];
                A[i*n+k] = A[i*n+pivrows[k]];
                A[i*n+pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}


void Matrix_Multiply(double* A, double* B, int m, int p, int n, double* C)
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
            C[n*i+j]=0;
            for (k=0;k<p;k++)
                C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}

void Matrix_Add(double* A, double* B, int m, int n, double* C)
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}

void Matrix_Transpose(double* A, int m, int n, double* C)
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[m*j+i]=A[n*i+j];
}
