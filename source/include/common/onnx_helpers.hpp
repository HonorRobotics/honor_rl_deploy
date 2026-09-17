#ifndef ONNX_HELPERS_HPP
#define ONNX_HELPERS_HPP

#include <iostream>
#include <vector>
#include <numeric>
#include <functional>
#include "onnxruntime_cxx_api.h"
#include "common/glog_sink.hpp"

class XbotOnnxRuntime {
public:
    XbotOnnxRuntime() : session_(nullptr), init_done_(false) {
        LOG(INFO) << "[xrobot_onnxruntime] onnxruntime construct";
    }

    ~XbotOnnxRuntime() {
        if (init_done_) {
            LOG(INFO) << "[xrobot_onnxruntime] onnxruntime deconstruct";
        }
    }

    // Init ONNX
    bool init_onnx_runtime(const std::string& model_path, int intra_op_num_threads = 1, int inter_op_num_threads = 1) {
        // path info
        LOG(INFO) << "[xrobot_onnxruntime] init onnxruntime with model path: " << model_path;

        // create ONNX Runtime environment
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntime");

        // create session object
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(intra_op_num_threads);
        session_options.SetInterOpNumThreads(inter_op_num_threads);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        session_options.DisableMemPattern();

        // Create the session
        session_ = Ort::Session(env, model_path.c_str(), session_options);
        if (!session_) {
            LOG(ERROR) << "[xrobot_onnxruntime] create ONNX session failed";
            return false;
        }

        // create allocator
        allocator_ = Ort::AllocatorWithDefaultOptions();

        // input init
        size_t num_input_nodes = session_.GetInputCount();
        for(size_t i = 0; i < num_input_nodes; i++) {
            auto inputname_ptr = session_.GetInputNameAllocated(i, allocator_);
            input_node_name_allocated_strings_.push_back(std::move(inputname_ptr));
            input_names_.push_back(input_node_name_allocated_strings_.back().get());
            LOG(INFO) << "[xrobot_onnxruntime] input[" << i << "] name: "<< input_names_[i];

            Ort::TypeInfo type_info = session_.GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            std::vector<int64_t> dims = tensor_info.GetShape();
            auto data_type = tensor_info.GetElementType();
            LOG(INFO) << "[xrobot_onnxruntime] input[" << i << "] data type: " << convert_onnx_tensor_type_to_string(data_type);

            for (auto &dim : dims) {
                // Set batch size unknown to 1
                dim = dim == -1 ? 1 : dim;
                LOG(INFO) << "[xrobot_onnxruntime] input[" << i << "] dim: "<< dim;
            }
            input_dims_.push_back(dims);
        }

        // output init
        size_t num_output_nodes = session_.GetOutputCount();
        for(size_t i = 0; i < num_output_nodes; i++) {
            auto outputname_ptr = session_.GetOutputNameAllocated(i, allocator_);
            output_node_name_allocated_strings_.push_back(std::move(outputname_ptr));
            output_names_.push_back(output_node_name_allocated_strings_.back().get());
            LOG(INFO) << "[xrobot_onnxruntime] output[" << i << "] name: " << output_names_[i];

            Ort::TypeInfo type_info = session_.GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            std::vector<int64_t> dims = tensor_info.GetShape();
            auto data_type = tensor_info.GetElementType();
            LOG(INFO) << "[xrobot_onnxruntime] output[" << i << "] data type: " << convert_onnx_tensor_type_to_string(data_type);

            for (auto &dim : dims) {
                // Set batch size unknown to 1
                dim = dim == -1 ? 1 : dim;
                LOG(INFO) << "[xrobot_onnxruntime] output[" << i << "] dim: "<< dim;
            }
            output_dims_.push_back(dims);
        }

        init_done_ = true;
        LOG(INFO) << "[xrobot_onnxruntime] init success!";
        return true;
    }

    bool infer(std::vector<float*> input_data, std::vector<float*> output_data) {
        if (!init_done_) {
            LOG(INFO) << "[xrobot_onnxruntime] ONNX Runtime not initialized";
            return false;
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for (size_t i = 0; i < input_data.size(); ++i) {
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                allocator_.GetInfo(), input_data[i],
                std::accumulate(input_dims_[i].begin(), input_dims_[i].end(), 1, std::multiplies<int64_t>()),
                input_dims_[i].data(), input_dims_[i].size());

            input_tensors.push_back(std::move(input_tensor));
        }

        // Execute inference
        auto output_tensors = session_.Run(
            Ort::RunOptions{nullptr},
            input_names_.data(), input_tensors.data(), input_tensors.size(),
            output_names_.data(), output_names_.size());

        // Copy output data
        for (size_t i = 0; i < output_tensors.size(); ++i) {
            float* output_ptr = output_tensors[i].GetTensorMutableData<float>();
            std::memcpy(output_data[i], output_ptr, sizeof(float) * std::accumulate(output_dims_[i].begin(), output_dims_[i].end(), 1, std::multiplies<int64_t>()));
        }

        return true;
    }

    std::vector<std::vector<int64_t>> get_input_dims() {
        return input_dims_;
    }

    std::vector<std::vector<int64_t>> get_output_dims() {
        return output_dims_;
    }

    std::vector<std::string> get_input_names() const {
        return std::vector<std::string>(input_names_.begin(), input_names_.end());
    }

    std::vector<std::string> get_output_names() const {
        return std::vector<std::string>(output_names_.begin(), output_names_.end());
    }


private:
    std::string convert_onnx_tensor_type_to_string(ONNXTensorElementDataType data_type) {
        switch (data_type) {
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
                return "Float";
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
                return "Int32";
            case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
                return "String";
            default:
                return "UnknownType";
        }
    }

    Ort::Session session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    std::vector<Ort::AllocatedStringPtr> input_node_name_allocated_strings_;
    std::vector<Ort::AllocatedStringPtr> output_node_name_allocated_strings_;
    std::vector<std::vector<int64_t>> input_dims_;
    std::vector<std::vector<int64_t>> output_dims_;
    bool init_done_;
};

#endif // ONNX_HELPERS_HPP
