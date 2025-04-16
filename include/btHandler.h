#ifndef BTHANDLER_H
#define BTHANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

int btstack_main(int argc, const char * argv[]);
const int* get_vector_data();
size_t get_vector_size();

#ifdef __cplusplus
}
#endif

#endif // BTHANDLER_H