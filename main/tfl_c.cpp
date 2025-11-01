#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "tfl_c.h"
#include "constants.h"

#include "model.h"
#include <esp_heap_caps.h>

/*
    Constrcutor
    Initializes the model and all required stuff
*/
TflC::TflC() {
    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                 "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // A good question
    // Reason?
    // Idk
    // Nothing else worked
    // Using heap memory and stack memory can be very expensive for micro controllers, and this works out of the million things I did
    // Please don't try optimizing this, ever
    // It's not necessary
    tensor_arena = idk;

    // Add the following functions
    static tflite::MicroMutableOpResolver<7> micro_op_resolver;
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddSoftmax();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddBatchMatMul();
    micro_op_resolver.AddAdd();

    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize
    );
    interpreter = &static_interpreter;

    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed");
        return;
    }

    // Get the input vector
    input = interpreter->input(0);
    // Get the output vector
    output = interpreter->output(0);
}

/*
    Run an inference
    Arr contains the scan values, to be put in the input vector
    Pred conatins the prediction values, which will be aquired from the output vector
    The return is the result with the highest probability
*/
int TflC::run_inf(float* arr, float* pred) {
    // Put the data in input
    for (int i = 0; i < 7; i++) {
        input->data.f[i] = arr[i];
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        MicroPrintf("Imvoke Failed");
        return -1;
    }

    // Find the max value for prediction
    int tens = 0;
    for (int i = 0; i < plastic_types_tot; i++) {
        pred[i] = output->data.f[i];
        tens = (pred[i] > pred[tens]) ? i : tens;
    }

    return tens;
}
