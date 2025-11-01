#ifndef TFL_C_H_
#define TFL_C_H_

/*
    The TensorFlow Lite Micro Class
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Forward declarations cos C/C++ sucks sometimes
namespace tflite {
    class Model;
    class MicroInterpreter;
}

class TfLiteTensor;


class TflC {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
    TfLiteTensor* output = nullptr;

    static constexpr int kTensorArenaSize = 5000;
    uint8_t *tensor_arena;
    uint8_t idk[5000];  // Look in cpp file for explanation

public:
    TflC();
    int run_inf(float* arr, float* pred);

};

#ifdef __cplusplus
}
#endif

#endif