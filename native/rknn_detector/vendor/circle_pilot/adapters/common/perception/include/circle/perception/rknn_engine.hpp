/**
 * @file rknn_engine.hpp
 * @brief RKNN C API inference wrapper for YOLOv8 on RK3588 NPU.
 *
 * Supports two operating modes:
 *   1. Standard mode: rknn_inputs_set / rknn_outputs_get with want_float=1.
 *   2. Zero-copy mode: Pre-allocated DMA buffers for both input and output.
 *      - Input:  caller writes directly into getInputBuffer().
 *      - Output: raw INT8 data in DMA buffers, caller does inline dequantization.
 */

#ifndef CIRCLE_PERCEPTION__RKNN_ENGINE_HPP_
#define CIRCLE_PERCEPTION__RKNN_ENGINE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace circle::perception {

enum class RknnLogLevel {
  Debug,
  Info,
  Warn,
  Error,
};

using RknnLogEnabledHandler = bool (*)(RknnLogLevel level);
using RknnLogHandler = void (*)(RknnLogLevel level, const char* message);

void setRknnLogHandler(RknnLogEnabledHandler enabled_handler,
                       RknnLogHandler log_handler);
bool rknnLogEnabled(RknnLogLevel level);
void rknnLog(RknnLogLevel level, const char* fmt, ...);

class RknnEngine {
 public:
  RknnEngine() = default;
  ~RknnEngine();

  RknnEngine(const RknnEngine&)            = delete;
  RknnEngine& operator=(const RknnEngine&) = delete;
  RknnEngine(RknnEngine&&)                 = delete;
  RknnEngine& operator=(RknnEngine&&)      = delete;

  /**
   * Load model and initialize NPU runtime.
   * @param model_path       Path to .rknn model file.
   * @param core_mask        NPU core mask (7 = triple-core for RK3588).
   * @param zero_copy_input  Allocate DMA input buffer (skip rknn_inputs_set copy).
   * @param zero_copy_output Allocate DMA output buffers (skip rknn_outputs_get dequant).
   * @return true on success.
   */
  bool init(const std::string& model_path, int core_mask = 7,
            bool zero_copy_input = false, bool zero_copy_output = false);
  void destroy();

  /** Standard inference: copies input_data via rknn_inputs_set. */
  bool run(const uint8_t* input_data, uint32_t input_size);

  /**
   * Zero-copy inference: data already in DMA buffers.
   * @param skip_input_sync  When true, skips rknn_mem_sync(TO_DEVICE) for the
   *        input buffer. Use when a DMA engine (e.g. RGA) wrote the input
   *        directly — no CPU cache flush needed.
   */
  bool runZeroCopy(bool skip_input_sync = false);

  // ── Input zero-copy ──
  uint8_t* getInputBuffer() const;
  uint32_t getInputBufferSize() const;
  int getInputDmaFd() const;
  bool isZeroCopyInput() const { return zero_copy_input_; }

  // ── Output access (standard mode: float32) ──
  const float* getOutputData(int index) const;

  // ── Output access (zero-copy mode: raw INT8 in DMA buffer) ──
  const int8_t* getOutputDataInt8(int index) const;
  bool isZeroCopyOutput() const { return zero_copy_output_; }

  // ── Output metadata ──
  int getOutputElements(int index) const;
  const uint32_t* getOutputShape(int index) const;
  int getOutputDims(int index) const;
  bool getOutputIsNHWC(int index) const;

  /** Quantization parameters for output tensor (valid for INT8 models). */
  int32_t getOutputZeroPoint(int index) const;
  float getOutputScale(int index) const;

  int numInputs() const;
  int numOutputs() const;
  bool isInitialized() const { return initialized_; }

  int inputWidth() const { return input_w_; }
  int inputHeight() const { return input_h_; }
  int inputChannels() const { return input_c_; }

 private:
  bool initialized_{false};
  uint64_t ctx_{0};

  int input_w_{0};
  int input_h_{0};
  int input_c_{0};

  uint32_t n_input_{0};
  uint32_t n_output_{0};

  // Zero-copy input
  bool zero_copy_input_{false};
  void* input_mem_{nullptr};       // rknn_tensor_mem*
  uint32_t input_mem_size_{0};
  bool input_needs_u8_to_s8_{false};  // true when pass_through=1 fallback is active
  int32_t input_zp_{0};               // native input zero point for the conversion

  // Zero-copy output
  bool zero_copy_output_{false};
  struct OutputMem {
    void* mem{nullptr};            // rknn_tensor_mem*
    uint32_t size{0};
  };
  std::vector<OutputMem> output_mems_;

  struct OutputInfo {
    uint32_t n_dims;
    uint32_t dims[16];
    uint32_t n_elems;
    uint32_t size;
    bool is_nhwc{false};
    // Quantization params (for affine asymmetric INT8)
    int32_t zp{0};
    float scale{1.0f};
    int type{0};                   // rknn_tensor_type
    int fmt{0};                    // rknn_tensor_format
  };
  std::vector<OutputInfo> output_info_;

  // Standard mode outputs (managed by rknn_outputs_get/release)
  struct OutputBuf {
    void* buf{nullptr};
    uint32_t size{0};
  };
  std::vector<OutputBuf> last_outputs_;
  bool outputs_valid_{false};

  void releaseOutputs();
  void destroyInputMem();
  void destroyOutputMems();
};

}  // namespace circle::perception

#endif  // CIRCLE_PERCEPTION__RKNN_ENGINE_HPP_
