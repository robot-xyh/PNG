/**
 * @file rknn_engine.cpp
 * @brief RKNN C API inference wrapper — supports standard and zero-copy modes.
 */

#include "circle/perception/rknn_engine.hpp"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <sys/stat.h>

#if USE_RKNN
#include <dlfcn.h>
#include <rknn_api.h>
#endif

#define RLOG(level, ...) \
  do { \
    if (circle::perception::rknnLogEnabled(level)) { \
      circle::perception::rknnLog(level, __VA_ARGS__); \
    } \
  } while (0)
#define DLOG(...) RLOG(circle::perception::RknnLogLevel::Debug, __VA_ARGS__)
#define ILOG(...) RLOG(circle::perception::RknnLogLevel::Info, __VA_ARGS__)
#define WLOG(...) RLOG(circle::perception::RknnLogLevel::Warn, __VA_ARGS__)
#define ELOG(...) RLOG(circle::perception::RknnLogLevel::Error, __VA_ARGS__)

namespace circle::perception {

namespace {

RknnLogEnabledHandler& rknnEnabledHandlerStorage() {
  static RknnLogEnabledHandler handler = nullptr;
  return handler;
}

RknnLogHandler& rknnLogHandlerStorage() {
  static RknnLogHandler handler = nullptr;
  return handler;
}

bool defaultRknnLogEnabled(RknnLogLevel level) {
  return level != RknnLogLevel::Debug;
}

void defaultRknnLog(RknnLogLevel, const char* message) {
  std::fprintf(stderr, "%s\n", message);
  std::fflush(stderr);
}

#if USE_RKNN
using RknnMemSyncFn =
    int (*)(rknn_context, rknn_tensor_mem*, rknn_mem_sync_mode);

RknnMemSyncFn resolveRknnMemSync() {
  static RknnMemSyncFn fn = reinterpret_cast<RknnMemSyncFn>(
      dlsym(RTLD_DEFAULT, "rknn_mem_sync"));
  return fn;
}

int rknnMemSyncOptional(rknn_context ctx, rknn_tensor_mem* mem,
                        rknn_mem_sync_mode mode, const char* label) {
  RknnMemSyncFn fn = resolveRknnMemSync();
  if (!fn) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      WLOG("[rknn] WARNING: rknn_mem_sync not exported by librknnrt; "
           "skipping explicit cache sync (%s)", label ? label : "unknown");
    }
    return RKNN_SUCC;
  }
  return fn(ctx, mem, mode);
}
#endif

}  // namespace

void setRknnLogHandler(RknnLogEnabledHandler enabled_handler,
                       RknnLogHandler log_handler) {
  rknnEnabledHandlerStorage() = enabled_handler;
  rknnLogHandlerStorage() = log_handler;
}

bool rknnLogEnabled(RknnLogLevel level) {
  RknnLogEnabledHandler handler = rknnEnabledHandlerStorage();
  if (handler) {
    return handler(level);
  }
  return defaultRknnLogEnabled(level);
}

void rknnLog(RknnLogLevel level, const char* fmt, ...) {
  std::array<char, 1024> buf{};
  va_list args;
  va_start(args, fmt);
  const int n = std::vsnprintf(buf.data(), buf.size(), fmt, args);
  va_end(args);
  if (n < 0) {
    return;
  }
  if (static_cast<size_t>(n) >= buf.size()) {
    buf.back() = '\0';
  }
  std::string message(buf.data());
  while (!message.empty() &&
         (message.back() == '\n' || message.back() == '\r')) {
    message.pop_back();
  }
  RknnLogHandler handler = rknnLogHandlerStorage();
  if (handler) {
    handler(level, message.c_str());
  } else {
    defaultRknnLog(level, message.c_str());
  }
}

#if !USE_RKNN

RknnEngine::~RknnEngine() {}
bool RknnEngine::init(const std::string&, int, bool, bool) { ELOG("[rknn] USE_RKNN=0 stub\n"); return false; }
void RknnEngine::destroy() {}
bool RknnEngine::run(const uint8_t*, uint32_t) { return false; }
bool RknnEngine::runZeroCopy(bool) { return false; }
uint8_t* RknnEngine::getInputBuffer() const { return nullptr; }
uint32_t RknnEngine::getInputBufferSize() const { return 0; }
int RknnEngine::getInputDmaFd() const { return -1; }
const float* RknnEngine::getOutputData(int) const { return nullptr; }
const int8_t* RknnEngine::getOutputDataInt8(int) const { return nullptr; }
int RknnEngine::getOutputElements(int) const { return 0; }
const uint32_t* RknnEngine::getOutputShape(int) const { return nullptr; }
int RknnEngine::getOutputDims(int) const { return 0; }
bool RknnEngine::getOutputIsNHWC(int) const { return false; }
int32_t RknnEngine::getOutputZeroPoint(int) const { return 0; }
float RknnEngine::getOutputScale(int) const { return 1.0f; }
int RknnEngine::numInputs() const { return 0; }
int RknnEngine::numOutputs() const { return 0; }
void RknnEngine::releaseOutputs() {}
void RknnEngine::destroyInputMem() {}
void RknnEngine::destroyOutputMems() {}

#else  // USE_RKNN

RknnEngine::~RknnEngine() { destroy(); }

bool RknnEngine::init(const std::string& model_path, int core_mask,
                      bool zero_copy_input, bool zero_copy_output)
{
  if (initialized_) return true;

  FILE* fp = fopen(model_path.c_str(), "rb");
  if (!fp) {
    ELOG("[rknn] ERROR: cannot open model file: %s\n", model_path.c_str());
    return false;
  }
  fseek(fp, 0, SEEK_END);
  long file_size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (file_size <= 0) {
    ELOG("[rknn] ERROR: model file is empty: %s\n", model_path.c_str());
    fclose(fp);
    return false;
  }
  std::vector<uint8_t> model_data(static_cast<size_t>(file_size));
  size_t read_n = fread(model_data.data(), 1, model_data.size(), fp);
  fclose(fp);
  if (read_n != model_data.size()) {
    ELOG("[rknn] ERROR: failed to read model file (got %zu / %zu)\n", read_n, model_data.size());
    return false;
  }

  ILOG("[rknn] Loading model: %s (%ld bytes)\n", model_path.c_str(), file_size);

  rknn_context ctx = 0;
  // Prefer NPU SRAM for eligible intermediate tensors (RK3588); reduces DDR traffic.
  const uint32_t init_flags = RKNN_FLAG_ENABLE_SRAM;
  int ret = rknn_init(&ctx, model_data.data(), static_cast<uint32_t>(model_data.size()), init_flags, nullptr);
  if (ret != RKNN_SUCC) {
    ELOG("[rknn] ERROR: rknn_init failed: %d (flags=0x%x)\n", ret, init_flags);
    return false;
  }
  DLOG("[rknn] rknn_init flags: RKNN_FLAG_ENABLE_SRAM (0x%x)\n", init_flags);
  ctx_ = ctx;

  ret = rknn_set_core_mask(ctx_, static_cast<rknn_core_mask>(core_mask));
  if (ret != RKNN_SUCC) {
    WLOG("[rknn] WARNING: rknn_set_core_mask(%d) failed: %d (non-fatal)\n", core_mask, ret);
  }

  rknn_sdk_version ver;
  memset(&ver, 0, sizeof(ver));
  ret = rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &ver, sizeof(ver));
  if (ret == RKNN_SUCC) {
    DLOG("[rknn] SDK api=%s driver=%s\n", ver.api_version, ver.drv_version);
  }

  rknn_input_output_num io_num;
  memset(&io_num, 0, sizeof(io_num));
  ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  if (ret != RKNN_SUCC) {
    ELOG("[rknn] ERROR: query IN_OUT_NUM failed: %d\n", ret);
    destroy();
    return false;
  }
  n_input_ = io_num.n_input;
  n_output_ = io_num.n_output;
  ILOG("[rknn] Model: %u input(s), %u output(s)\n", n_input_, n_output_);

  // ── Query input attributes ──
  rknn_tensor_attr in_attr;
  memset(&in_attr, 0, sizeof(in_attr));
  in_attr.index = 0;
  for (uint32_t i = 0; i < n_input_; ++i) {
    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.index = i;
    ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &attr, sizeof(attr));
    if (ret != RKNN_SUCC) {
      ELOG("[rknn] ERROR: query INPUT_ATTR[%u] failed: %d\n", i, ret);
      destroy();
      return false;
    }
    if (rknnLogEnabled(RknnLogLevel::Debug)) {
      std::ostringstream oss;
      oss << "[rknn] Input[" << i << "]: dims=[";
      for (uint32_t d = 0; d < attr.n_dims; ++d) {
        oss << attr.dims[d] << (d + 1 < attr.n_dims ? "," : "");
      }
      oss << "], fmt=" << get_format_string(attr.fmt)
          << ", type=" << get_type_string(attr.type)
          << ", size=" << attr.size
          << ", size_with_stride=" << attr.size_with_stride;
      DLOG("%s", oss.str().c_str());
    }

    if (i == 0) {
      in_attr = attr;
      if (attr.n_dims == 4) {
        if (attr.fmt == RKNN_TENSOR_NHWC) {
          input_h_ = static_cast<int>(attr.dims[1]);
          input_w_ = static_cast<int>(attr.dims[2]);
          input_c_ = static_cast<int>(attr.dims[3]);
        } else {
          input_c_ = static_cast<int>(attr.dims[1]);
          input_h_ = static_cast<int>(attr.dims[2]);
          input_w_ = static_cast<int>(attr.dims[3]);
        }
      }
    }
  }
  ILOG("[rknn] Model input: %dx%dx%d\n", input_w_, input_h_, input_c_);

  // ── Query output attributes ──
  output_info_.resize(n_output_);
  std::vector<rknn_tensor_attr> out_attrs(n_output_);
  for (uint32_t i = 0; i < n_output_; ++i) {
    memset(&out_attrs[i], 0, sizeof(rknn_tensor_attr));
    out_attrs[i].index = i;
    ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attrs[i], sizeof(rknn_tensor_attr));
    if (ret != RKNN_SUCC) {
      ELOG("[rknn] ERROR: query OUTPUT_ATTR[%u] failed: %d\n", i, ret);
      destroy();
      return false;
    }
    auto& attr = out_attrs[i];
    output_info_[i].n_dims = attr.n_dims;
    memcpy(output_info_[i].dims, attr.dims, sizeof(attr.dims));
    output_info_[i].n_elems = attr.n_elems;
    output_info_[i].size = attr.size;
    output_info_[i].is_nhwc = (attr.fmt == RKNN_TENSOR_NHWC);
    output_info_[i].zp = attr.zp;
    output_info_[i].scale = attr.scale;
    output_info_[i].type = attr.type;
    output_info_[i].fmt = attr.fmt;

    if (rknnLogEnabled(RknnLogLevel::Debug)) {
      std::ostringstream oss;
      oss << "[rknn] Output[" << i << "]: dims=[";
      for (uint32_t d = 0; d < attr.n_dims; ++d) {
        oss << attr.dims[d] << (d + 1 < attr.n_dims ? "," : "");
      }
      oss << "], fmt=" << get_format_string(attr.fmt)
          << ", type=" << get_type_string(attr.type)
          << ", n_elems=" << attr.n_elems
          << ", zp=" << attr.zp
          << ", scale=" << attr.scale;
      DLOG("%s", oss.str().c_str());
    }
  }

  last_outputs_.resize(n_output_);

  // ── Setup zero-copy input ──
  // Query NATIVE input attributes — required by rknn_set_io_mem for zero-copy
  rknn_tensor_attr native_in_attr;
  memset(&native_in_attr, 0, sizeof(native_in_attr));
  native_in_attr.index = 0;
  ret = rknn_query(ctx_, RKNN_QUERY_NATIVE_NHWC_INPUT_ATTR, &native_in_attr, sizeof(native_in_attr));
  if (ret != RKNN_SUCC) {
    DLOG("[rknn] query NATIVE_NHWC_INPUT_ATTR failed: %d, trying NATIVE_INPUT_ATTR\n", ret);
    ret = rknn_query(ctx_, RKNN_QUERY_NATIVE_INPUT_ATTR, &native_in_attr, sizeof(native_in_attr));
  }
  if (ret == RKNN_SUCC) {
    if (rknnLogEnabled(RknnLogLevel::Debug)) {
      std::ostringstream oss;
      oss << "[rknn] Native input: dims=[";
      for (uint32_t d = 0; d < native_in_attr.n_dims; ++d) {
        oss << native_in_attr.dims[d]
            << (d + 1 < native_in_attr.n_dims ? "," : "");
      }
      oss << "], fmt=" << get_format_string(native_in_attr.fmt)
          << ", type=" << get_type_string(native_in_attr.type)
          << ", size=" << native_in_attr.size
          << ", w_stride=" << native_in_attr.w_stride
          << ", zp=" << native_in_attr.zp
          << ", scale=" << native_in_attr.scale;
      DLOG("%s", oss.str().c_str());
    }
  } else {
    WLOG("[rknn] WARNING: could not query native input attrs, using regular attrs\n");
    native_in_attr = in_attr;
  }

  zero_copy_input_ = zero_copy_input;
  if (zero_copy_input_) {
    uint32_t alloc_size = native_in_attr.size_with_stride > 0
        ? native_in_attr.size_with_stride : native_in_attr.size;
    if (alloc_size == 0) {
      alloc_size = static_cast<uint32_t>(input_w_) * input_h_ * input_c_;
    }

    rknn_tensor_mem* mem = rknn_create_mem(ctx_, alloc_size);
    if (!mem) {
      ELOG("[rknn] ERROR: rknn_create_mem failed for input (%u bytes)\n", alloc_size);
      zero_copy_input_ = false;
    } else {
      input_mem_ = mem;
      input_mem_size_ = alloc_size;

      // Start from native attrs (preserves w_stride, h_stride, qnt_type, zp,
      // scale and other fields that rknn_set_io_mem may check internally).
      // Set pass_through=0 so runtime converts our UINT8 data to INT8.
      rknn_tensor_attr io_attr = native_in_attr;
      io_attr.size = alloc_size;
      io_attr.size_with_stride = alloc_size;
      io_attr.pass_through = 0;
      io_attr.type = RKNN_TENSOR_UINT8;
      io_attr.fmt = RKNN_TENSOR_NHWC;

      ret = rknn_set_io_mem(ctx_, mem, &io_attr);
      if (ret != RKNN_SUCC) {
        WLOG("[rknn] rknn_set_io_mem(pass_through=0, UINT8) failed: %d, "
             "falling back to pass_through=1 (native INT8)\n", ret);
        // Fallback: pass_through=1, caller data is treated as native INT8.
        // We will convert UINT8→INT8 ourselves before each inference.
        rknn_tensor_attr fb_attr = native_in_attr;
        fb_attr.size = alloc_size;
        fb_attr.size_with_stride = alloc_size;
        fb_attr.pass_through = 1;
        ret = rknn_set_io_mem(ctx_, mem, &fb_attr);
        if (ret != RKNN_SUCC) {
          ELOG("[rknn] ERROR: rknn_set_io_mem(pass_through=1) also failed: %d\n", ret);
          rknn_destroy_mem(ctx_, mem);
          input_mem_ = nullptr;
          input_mem_size_ = 0;
          zero_copy_input_ = false;
        } else {
          input_needs_u8_to_s8_ = true;
          input_zp_ = native_in_attr.zp;
          DLOG("[rknn] Zero-copy input enabled (pass_through=1, manual UINT8→INT8, zp=%d): "
               "%u bytes @ %p\n", input_zp_, alloc_size, mem->virt_addr);
        }
      } else {
        DLOG("[rknn] Zero-copy input enabled (pass_through=0, UINT8): %u bytes @ %p\n",
             alloc_size, mem->virt_addr);
      }
    }
  }

  // ── Query native output attributes for zero-copy ──
  std::vector<rknn_tensor_attr> native_out_attrs(n_output_);
  {
    bool native_query_ok = true;
    for (uint32_t i = 0; i < n_output_; ++i) {
      memset(&native_out_attrs[i], 0, sizeof(rknn_tensor_attr));
      native_out_attrs[i].index = i;
      ret = rknn_query(ctx_, RKNN_QUERY_NATIVE_NHWC_OUTPUT_ATTR,
                        &native_out_attrs[i], sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC) {
        ret = rknn_query(ctx_, RKNN_QUERY_NATIVE_OUTPUT_ATTR,
                          &native_out_attrs[i], sizeof(rknn_tensor_attr));
      }
      if (ret != RKNN_SUCC) {
        native_query_ok = false;
        native_out_attrs[i] = out_attrs[i];
      }
    }
    if (native_query_ok) {
      DLOG("[rknn] Native output attrs queried OK\n");
    }
  }

  // ── Setup zero-copy output ──
  zero_copy_output_ = zero_copy_output;
  if (zero_copy_output_) {
    output_mems_.resize(n_output_);
    bool all_ok = true;
    for (uint32_t i = 0; i < n_output_; ++i) {
      auto& attr = out_attrs[i];
      auto& nattr = native_out_attrs[i];
      uint32_t alloc_size = nattr.size_with_stride > 0
          ? nattr.size_with_stride : (nattr.size > 0 ? nattr.size : attr.n_elems);
      if (alloc_size == 0) {
        alloc_size = attr.n_elems;
      }

      rknn_tensor_mem* mem = rknn_create_mem(ctx_, alloc_size);
      if (!mem) {
        ELOG("[rknn] ERROR: rknn_create_mem failed for output[%u] (%u bytes)\n", i, alloc_size);
        all_ok = false;
        break;
      }
      output_mems_[i].mem = mem;
      output_mems_[i].size = alloc_size;

      // Use native attributes with pass_through=1
      rknn_tensor_attr io_attr = nattr;
      io_attr.size = alloc_size;
      io_attr.size_with_stride = alloc_size;
      io_attr.pass_through = 1;

      ret = rknn_set_io_mem(ctx_, mem, &io_attr);
      if (ret != RKNN_SUCC) {
        WLOG("[rknn] rknn_set_io_mem(output[%u], native, pass_through=1) failed: %d, "
             "retrying pass_through=0\n", i, ret);
        io_attr = nattr;
        io_attr.size = alloc_size;
        io_attr.size_with_stride = alloc_size;
        io_attr.pass_through = 0;
        ret = rknn_set_io_mem(ctx_, mem, &io_attr);
      }
      if (ret != RKNN_SUCC) {
        ELOG("[rknn] ERROR: rknn_set_io_mem for output[%u] failed: %d\n", i, ret);
        all_ok = false;
        break;
      }
      // When pass_through=1, data is in native quantization format.
      // Use native zp/scale/dims/fmt for correct dequantization and layout parsing.
      output_info_[i].zp = nattr.zp;
      output_info_[i].scale = nattr.scale;
      output_info_[i].is_nhwc = (nattr.fmt == RKNN_TENSOR_NHWC);
      output_info_[i].n_dims = nattr.n_dims;
      memcpy(output_info_[i].dims, nattr.dims, sizeof(nattr.dims));
      output_info_[i].fmt = nattr.fmt;
      DLOG("[rknn] Zero-copy output[%u]: %u bytes @ %p (type=%s, zp=%d, scale=%.6f, native_zp=%d, native_scale=%.6f, pass_through=%d)\n",
           i, alloc_size, mem->virt_addr,
           get_type_string(attr.type), attr.zp, attr.scale,
           nattr.zp, nattr.scale, io_attr.pass_through);
    }
    if (!all_ok) {
      WLOG("[rknn] WARNING: zero-copy output setup failed, falling back to standard mode\n");
      destroyOutputMems();
      zero_copy_output_ = false;
    }
  }

  initialized_ = true;
  ILOG("[rknn] Init OK (core_mask=%d, zc_in=%s, zc_out=%s)\n",
       core_mask,
       zero_copy_input_ ? "true" : "false",
       zero_copy_output_ ? "true" : "false");
  return true;
}

void RknnEngine::destroyInputMem()
{
  if (input_mem_ && ctx_ != 0) {
    rknn_destroy_mem(ctx_, static_cast<rknn_tensor_mem*>(input_mem_));
  }
  input_mem_ = nullptr;
  input_mem_size_ = 0;
  zero_copy_input_ = false;
  input_needs_u8_to_s8_ = false;
  input_zp_ = 0;
}

void RknnEngine::destroyOutputMems()
{
  for (auto& om : output_mems_) {
    if (om.mem && ctx_ != 0) {
      rknn_destroy_mem(ctx_, static_cast<rknn_tensor_mem*>(om.mem));
    }
    om.mem = nullptr;
    om.size = 0;
  }
  output_mems_.clear();
}

void RknnEngine::destroy()
{
  releaseOutputs();
  output_info_.clear();
  last_outputs_.clear();
  destroyInputMem();
  destroyOutputMems();

  if (ctx_ != 0) {
    rknn_destroy(ctx_);
    ctx_ = 0;
  }
  initialized_ = false;
  n_input_ = 0;
  n_output_ = 0;
  zero_copy_output_ = false;
}

bool RknnEngine::run(const uint8_t* input_data, uint32_t input_size)
{
  if (!initialized_ || !input_data || input_size == 0) return false;

  releaseOutputs();

  rknn_input inputs[1];
  memset(inputs, 0, sizeof(inputs));
  inputs[0].index = 0;
  inputs[0].buf = const_cast<uint8_t*>(input_data);
  inputs[0].size = input_size;
  inputs[0].pass_through = 0;
  inputs[0].type = RKNN_TENSOR_UINT8;
  inputs[0].fmt = RKNN_TENSOR_NHWC;

  int ret = rknn_inputs_set(ctx_, 1, inputs);
  if (ret != RKNN_SUCC) {
    ELOG("[rknn] ERROR: rknn_inputs_set failed: %d\n", ret);
    return false;
  }

  rknn_run_extend extend;
  memset(&extend, 0, sizeof(extend));
  extend.non_block = 0;
  extend.timeout_ms = 500;
  ret = rknn_run(ctx_, &extend);
  if (ret != RKNN_SUCC) {
    ELOG("[rknn] ERROR: rknn_run failed: %d\n", ret);
    return false;
  }

  if (!zero_copy_output_) {
    std::vector<rknn_output> outputs(n_output_);
    for (uint32_t i = 0; i < n_output_; ++i) {
      memset(&outputs[i], 0, sizeof(rknn_output));
      outputs[i].want_float = 1;
      outputs[i].is_prealloc = 0;
      outputs[i].index = i;
    }
    ret = rknn_outputs_get(ctx_, n_output_, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) {
      ELOG("[rknn] ERROR: rknn_outputs_get failed: %d\n", ret);
      return false;
    }
    for (uint32_t i = 0; i < n_output_; ++i) {
      last_outputs_[i].buf = outputs[i].buf;
      last_outputs_[i].size = outputs[i].size;
    }
    outputs_valid_ = true;
  }
  // zero_copy_output: data is already in output_mems_[i].mem->virt_addr
  return true;
}

bool RknnEngine::runZeroCopy(bool skip_input_sync)
{
  if (!initialized_) return false;

  releaseOutputs();

  if (zero_copy_input_ && input_mem_) {
    auto* mem = static_cast<rknn_tensor_mem*>(input_mem_);

    // When pass_through=1 fallback is active, convert UINT8→INT8 in-place.
    if (input_needs_u8_to_s8_) {
      // If DMA engine wrote the buffer, invalidate CPU cache first.
      if (skip_input_sync) {
        (void)rknnMemSyncOptional(ctx_, mem, RKNN_MEMORY_SYNC_FROM_DEVICE,
                                  "input FROM_DEVICE");
      }

      auto* buf = static_cast<uint8_t*>(mem->virt_addr);
      if (buf) {
        const int32_t offset = 128 + input_zp_;
        if (offset == 0) {
          for (uint32_t i = 0; i < input_mem_size_; ++i)
            buf[i] ^= 0x80;
        } else {
          for (uint32_t i = 0; i < input_mem_size_; ++i) {
            int32_t v = static_cast<int32_t>(buf[i]) - 128 + offset;
            buf[i] = static_cast<uint8_t>(
                std::max(-128, std::min(127, v)) & 0xFF);
          }
        }
      }
      // Always flush after in-place modification
      int ret = rknnMemSyncOptional(ctx_, mem, RKNN_MEMORY_SYNC_TO_DEVICE,
                                    "input TO_DEVICE");
      if (ret != RKNN_SUCC) {
        WLOG("[rknn] WARNING: rknn_mem_sync(input, TO_DEVICE) failed: %d\n", ret);
      }
    } else if (!skip_input_sync) {
      // Standard path: CPU wrote buffer, flush cache to device
      int ret = rknnMemSyncOptional(ctx_, mem, RKNN_MEMORY_SYNC_TO_DEVICE,
                                    "input TO_DEVICE");
      if (ret != RKNN_SUCC) {
        WLOG("[rknn] WARNING: rknn_mem_sync(input, TO_DEVICE) failed: %d\n", ret);
      }
    }
  }

  rknn_run_extend extend;
  memset(&extend, 0, sizeof(extend));
  extend.non_block = 0;
  extend.timeout_ms = 500;
  int ret = rknn_run(ctx_, &extend);
  if (ret != RKNN_SUCC) {
    ELOG("[rknn] ERROR: rknn_run failed: %d\n", ret);
    return false;
  }

  if (zero_copy_output_) {
    // Sync output caches from device so CPU can read
    for (uint32_t i = 0; i < n_output_; ++i) {
      if (output_mems_[i].mem) {
        auto* mem = static_cast<rknn_tensor_mem*>(output_mems_[i].mem);
        ret = rknnMemSyncOptional(ctx_, mem, RKNN_MEMORY_SYNC_FROM_DEVICE,
                                  "output FROM_DEVICE");
        if (ret != RKNN_SUCC) {
          WLOG("[rknn] WARNING: rknn_mem_sync(output[%u], FROM_DEVICE) failed: %d\n", i, ret);
        }
      }
    }
  } else {
    // Standard output path
    std::vector<rknn_output> outputs(n_output_);
    for (uint32_t i = 0; i < n_output_; ++i) {
      memset(&outputs[i], 0, sizeof(rknn_output));
      outputs[i].want_float = 1;
      outputs[i].is_prealloc = 0;
      outputs[i].index = i;
    }
    ret = rknn_outputs_get(ctx_, n_output_, outputs.data(), nullptr);
    if (ret != RKNN_SUCC) {
      ELOG("[rknn] ERROR: rknn_outputs_get failed: %d\n", ret);
      return false;
    }
    for (uint32_t i = 0; i < n_output_; ++i) {
      last_outputs_[i].buf = outputs[i].buf;
      last_outputs_[i].size = outputs[i].size;
    }
    outputs_valid_ = true;
  }
  return true;
}

uint8_t* RknnEngine::getInputBuffer() const
{
  if (!zero_copy_input_ || !input_mem_) return nullptr;
  return static_cast<uint8_t*>(static_cast<rknn_tensor_mem*>(input_mem_)->virt_addr);
}

uint32_t RknnEngine::getInputBufferSize() const
{
  return input_mem_size_;
}

int RknnEngine::getInputDmaFd() const
{
  if (!zero_copy_input_ || !input_mem_) return -1;
  return static_cast<rknn_tensor_mem*>(input_mem_)->fd;
}

void RknnEngine::releaseOutputs()
{
  if (!outputs_valid_ || ctx_ == 0) return;

  std::vector<rknn_output> outputs(n_output_);
  for (uint32_t i = 0; i < n_output_; ++i) {
    memset(&outputs[i], 0, sizeof(rknn_output));
    outputs[i].want_float = 1;
    outputs[i].is_prealloc = 0;
    outputs[i].index = i;
    outputs[i].buf = last_outputs_[i].buf;
    outputs[i].size = last_outputs_[i].size;
  }
  rknn_outputs_release(ctx_, n_output_, outputs.data());

  for (uint32_t i = 0; i < n_output_; ++i) {
    last_outputs_[i].buf = nullptr;
    last_outputs_[i].size = 0;
  }
  outputs_valid_ = false;
}

const float* RknnEngine::getOutputData(int index) const
{
  if (zero_copy_output_) return nullptr;  // use getOutputDataInt8 instead
  if (!outputs_valid_ || index < 0 || static_cast<uint32_t>(index) >= n_output_) return nullptr;
  return static_cast<const float*>(last_outputs_[index].buf);
}

const int8_t* RknnEngine::getOutputDataInt8(int index) const
{
  if (!zero_copy_output_ || index < 0 || static_cast<uint32_t>(index) >= n_output_) return nullptr;
  if (!output_mems_[index].mem) return nullptr;
  return static_cast<const int8_t*>(static_cast<rknn_tensor_mem*>(output_mems_[index].mem)->virt_addr);
}

int RknnEngine::getOutputElements(int index) const
{
  if (index < 0 || static_cast<uint32_t>(index) >= n_output_) return 0;
  return static_cast<int>(output_info_[index].n_elems);
}

const uint32_t* RknnEngine::getOutputShape(int index) const
{
  if (index < 0 || static_cast<uint32_t>(index) >= n_output_) return nullptr;
  return output_info_[index].dims;
}

int RknnEngine::getOutputDims(int index) const
{
  if (index < 0 || static_cast<uint32_t>(index) >= n_output_) return 0;
  return static_cast<int>(output_info_[index].n_dims);
}

bool RknnEngine::getOutputIsNHWC(int index) const
{
  if (index < 0 || static_cast<uint32_t>(index) >= n_output_) return false;
  return output_info_[index].is_nhwc;
}

int32_t RknnEngine::getOutputZeroPoint(int index) const
{
  if (index < 0 || static_cast<uint32_t>(index) >= n_output_) return 0;
  return output_info_[index].zp;
}

float RknnEngine::getOutputScale(int index) const
{
  if (index < 0 || static_cast<uint32_t>(index) >= n_output_) return 1.0f;
  return output_info_[index].scale;
}

int RknnEngine::numInputs() const { return static_cast<int>(n_input_); }
int RknnEngine::numOutputs() const { return static_cast<int>(n_output_); }

#endif  // USE_RKNN

}  // namespace circle::perception
