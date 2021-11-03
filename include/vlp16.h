
// This file is a part of MRNIU/VLP-16
// (https://github.com/MRNIU/VLP-16).
//
// vlp16.h for MRNIU/VLP-16.

#ifndef _VLP16_H_
#define _VLP16_H_

#include "stdint.h"
#include "vector"

// channel 数量
static constexpr const uint32_t CHANNEL = 16;
// 方位区域数量
static constexpr const uint32_t ROWS = 12;
// UDP接收到的数据大小
static constexpr const uint32_t DATA_SIZE = 1206;
// 垂直方向上的角度
static constexpr const int VerticalAngles[] = {
    -15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

// TODO: 字节序处理
// 最小数据组
class vlp16_dis_ref_datablock_t {
private:
    // 距离
    uint16_t distance;
    // 反射率
    uint8_t reflectivity;

protected:
public:
    // 获取距离
    uint16_t get_distance(void) const {
        uint16_t tmp = distance << 8;
        tmp += distance & 0xFF;
        return tmp * 2;
    }
    // 获取反射率
    uint16_t get_refl(void);
};

// 行数据
class vlp16_row_datablock_t {
private:
    static constexpr const uint16_t FFEE = 0xFFEE;
    // 前导 0xFFEE
    uint16_t head;
    // 方位角
    uint16_t azimuth;
    // 距离强度数据
    // 一次回波
    vlp16_dis_ref_datablock_t dis_ref_datablock1[CHANNEL];
    // 二次回波
    vlp16_dis_ref_datablock_t dis_ref_datablock2[CHANNEL];

protected:
public:
    vlp16_row_datablock_t(void);
    ~vlp16_row_datablock_t(void);
    // 获取方位角
    uint16_t get_azimuth(void) const {
        uint16_t tmp = azimuth << 8;
        tmp += azimuth & 0xFF;
        return tmp / 100.0;
    }
    // 获取第 i 组距离
    uint16_t get_distance(int i) {
        return dis_ref_datablock1[i].get_distance();
    }
};

// 最上层数据块
class vlp16_datablock_t {
private:
    // 12 组数据
    vlp16_row_datablock_t row_datablock[ROWS];
    // 结束信息
    uint8_t end[6];

protected:
public:
    vlp16_datablock_t(void);
    ~vlp16_datablock_t(void);
    bool ava(void);
    // 获取指定 row
    vlp16_row_datablock_t get_row(int row);
};

// 雷达对象
class vlp16 {
private:
    // 当前读取到的数据
    vlp16_datablock_t data;
    // 获取 data 数据
    void read_data(void);
    // 规范角度
    void normalize_angle(double &angle);
    // 获取指定水平方位的数据
    std::vector<vlp16_datablock_t> vlp16::get_data(double azimuthFrom,
                                                   double azimuthTo);

protected:
public:
    vlp16(void);
    ~vlp16(void);
};

#endif
