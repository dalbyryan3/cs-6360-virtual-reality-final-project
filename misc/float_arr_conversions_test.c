#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')


void print_f_arr(float *arr, size_t len)
{
    printf("Float representation of float array\n");
    for (int i = 0; i < len; i++)
    {
        printf("Element %d: %f", i, arr[i]);
        printf("\n");
    }
    printf("\n");
    printf("\n");
}
void print_u_arr(uint8_t *arr, size_t len)
{
    printf("Binary representation of uint8_t array\n");
    for (int i = 0; i < len; i++)
    {
        printf("Element %d:", i);
        printf(" "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(arr[i]));
        printf("\n");
    }
    printf("\n");
    printf("\n");
}
void print_f_from_u_arr(uint8_t *arr, size_t arr_len, float *arr_f, size_t arr_f_len)
{
  memcpy(arr_f, arr, arr_len);
  printf("Converted uint8_t array to float array\n");
  print_f_arr(arr_f, arr_f_len);
}

int main()
{
    float test[3] = {0.8, 9.4, 98032.5};
    printf("Size %lu\n", sizeof(test));
    uint8_t *test_b = (uint8_t *) test;
    print_f_arr(test, 3);
    print_u_arr(test_b, sizeof(test));
    float test_from_u[3] = {0};
    print_f_from_u_arr(test_b, sizeof(test), test_from_u, sizeof(test)/4);

    return 0;
}


