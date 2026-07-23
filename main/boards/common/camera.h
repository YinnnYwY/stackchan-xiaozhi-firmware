#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <vector>
#include <cstdint>

class Camera {
public:
    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op
    virtual std::string Explain(const std::string& question) = 0;
    // 本地 JPEG 编码(不经过云端 explain_url_):给局域网 HTTP 接口(/camera)用,
    // 例如 Mac 上跑 MediaPipe 人脸追踪时轮询拿照片。默认不支持,子类按需实现。
    virtual bool CaptureJpeg(std::vector<uint8_t>& out_jpeg) { return false; }
};

#endif // CAMERA_H
