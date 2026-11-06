#pragma once

// 发布前画面旋转内核:供各相机节点共用(hk_camera_node / iray_camera_node)。
// 只依赖标准库,不含任何 ROS 或相机 SDK 头,可安全被多个组件共用。

#include <algorithm>
#include <cstddef>

namespace ros_camera
{

// 发布前是否把画面内容逆时针旋转 90°（相机物理安装旋转 90° 时需要）。
// 置 false 恢复原始方向直发；改方向/取消旋转只改这一处即可。
// 用 inline constexpr 而非 static constexpr：两个相机节点各自编译成独立组件，
// 都会 include 本头文件，inline 保证多翻译单元下的定义唯一且共享同一常量。
inline constexpr bool kRotateImageCcw90 = false;

// 单遍 BGR24 逆时针 90° 旋转内核（与 cv::ROTATE_90_COUNTERCLOCKWISE 同约定）。
// 映射公式：out(i, j) = in(j, w-1-i)，即原图顶边 -> 输出左边，宽高互换。
// 自检样例："ab/cd" 旋转后首行应为 "b d"。
// 性能：目标行按 kBlockRows 分块，块内源像素沿行连续读取（突发 <=192B），
// 目标侧每个 64B cacheline 的活动窗口仅约 kBlockRows 行，常驻 L1/L2；
// DRAM 流量约等于读+写各一遍，无逐列访问的 cacheline 放大。无临时缓冲、无分配。
inline void rotateBgr90Ccw(
    const unsigned char * in, unsigned char * out, const int w, const int h)
{
    constexpr int kBlockRows = 64;  // 输出行分块数；末块由 min() 收尾，任意宽高均可
    for (int i0 = 0; i0 < w; i0 += kBlockRows) {
        const int i1 = std::min(i0 + kBlockRows, w);
        for (int j = 0; j < h; ++j) {
            // 本块对应原图第 j 行上从右往左的 (i1-i0) 个像素列
            const unsigned char * src_col =
                in + static_cast<std::size_t>(j) * 3U * static_cast<std::size_t>(w) +
                3U * static_cast<std::size_t>(w - 1 - i0);
            // 输出第 j 列、行 i0..i1-1（每行进 j*3 字节）
            unsigned char * dst =
                out + (static_cast<std::size_t>(i0) * static_cast<std::size_t>(h) +
                       static_cast<std::size_t>(j)) * 3U;
            const std::size_t dst_row_stride = 3U * static_cast<std::size_t>(h);
            for (int i = i0; i < i1; ++i) {
                dst[0] = src_col[0];
                dst[1] = src_col[1];
                dst[2] = src_col[2];
                src_col -= 3;  // 源列左移一格 == 输出行下移一格
                dst += dst_row_stride;
            }
        }
    }
}

}  // namespace ros_camera
