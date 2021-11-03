
// This file is a part of MRNIU/VLP-16
// (https://github.com/MRNIU/VLP-16).
//
// vlp16.cpp for MRNIU/VLP-16.

#include "vlp16.h"
#include "assert.h"

void vlp16::read_data(void) {

    return;
}

void vlp16::normalize_angle(double &angle) {
    while (angle < 0) {
        angle += 360;
    }
    while (angle >= 360) {
        angle -= 360;
    }
    return;
}

std::vector<vlp16_datablock_t> vlp16::get_data(double azimuthFrom,
                                               double azimuthTo) {
    normalize_angle(azimuthFrom);
    normalize_angle(azimuthTo);
    std::vector<vlp16_datablock_t> LidarFormatedData;
    double                         angle;
    double                         oldAngle;
    // 是否开始记录
    bool recording = false;
    // 更新 data
    read_data();
    if (data.Length != DATA_SIZE) {
        udp.Close();
    }
    while (true) {
        // 获取当前数据的起始角度
        angle = data.get_row(0).get_azimuth();
        // 如果起始角度在 From To 范围内
        if (angle >= azimuthFrom && angle <= azimuthTo) {
            // 记录
            LidarFormatedData.push_back(data);
        }
        // 如果超出了则返回
        if (angle > azimuthTo) {
            return LidarFormatedData;
        }
        // 获取下一组数据
        read_data();
    }
}

class Lidar {

    uint8_t[] data = null;

    uint8_t[] prevLidarData = null;

    ushort Port = 2368;

    Lidar(ushort port = 2368) {
        Port = port;
    }

    uint8_t[][] GetRawData(int n) {
        uint8_t[][] data = new uint8_t[n][];
        //Чтение данных по UDP
        UdpClient  receivingUdpClient = new UdpClient(Port);
        IPEndPoint RemoteIpEndPoint   = null;
        for (int i = 0; i < n; ++i) {
            data[i] = receivingUdpClient.Receive(ref RemoteIpEndPoint);
        }
        receivingUdpClient.Close();
        return data;
    }

    uint8_t[] GetRawPacket() {
        uint8_t[] data;

        UdpClient receivingUdpClient = new UdpClient(Port);

        IPEndPoint RemoteIpEndPoint = null;
        data = receivingUdpClient.Receive(ref RemoteIpEndPoint);
        receivingUdpClient.Close();
        return data;
    }

    void FindMaximum(out int dist, out double azimuth, out int channel) {
        double azimuthFrom = 0;
        double azimuthTo   = 359.99;
        int[] channelsList = new int[16];
        for (int i = 0; i < channelsList.Length; ++i) {
            channelsList[i] = i;
        }
        FindExtremum(out dist, out azimuth, out channel, true, azimuthFrom,
                     azimuthTo, channelsList);
    }

    void FindMaximum(out int dist, out double azimuth, out int channel,
                     double azimuthFrom, double azimuthTo) {
        int[] channelsList = new int[16];
        for (int i = 0; i < channelsList.Length; ++i) {
            channelsList[i] = i;
        }
        FindExtremum(out dist, out azimuth, out channel, true, azimuthFrom,
                     azimuthTo, channelsList);
    }

    void FindMaximum(out int dist, out double azimuth, out int channel,
                     double azimuthFrom, double azimuthTo, int[] channelsList) {
        if (channelsList == null) {
            throw new NullReferenceException("List of channels is null");
        }
        for (int i = 0; i < channelsList.Length; ++i) {
            if (channelsList[i] > 15 || channelsList[i] < 0) {
                throw new IndexOutOfRangeException(
                    "List of channels contains incorrect channel number");
            }
        }
        FindExtremum(out dist, out azimuth, out channel, true, azimuthFrom,
                     azimuthTo, channelsList);
    }

    void FindMinimum(out int dist, out double azimuth, out int channel) {
        double azimuthFrom = 0;
        double azimuthTo   = 359.99;
        int[] channelsList = new int[16];
        for (int i = 0; i < channelsList.Length; ++i) {
            channelsList[i] = i;
        }
        FindExtremum(out dist, out azimuth, out channel, false, azimuthFrom,
                     azimuthTo, channelsList);
    }

    void FindMinimum(out int dist, out double azimuth, out int channel,
                     double azimuthFrom, double azimuthTo) {
        int[] channelsList = new int[16];
        for (int i = 0; i < channelsList.Length; ++i) {
            channelsList[i] = i;
        }
        FindExtremum(out dist, out azimuth, out channel, false, azimuthFrom,
                     azimuthTo, channelsList);
    }

    void FindMinimum(out int dist, out double azimuth, out int channel,
                     double azimuthFrom, double azimuthTo, int[] channelsList) {
        if (channelsList == null) {
            throw new NullReferenceException("List of channels is null");
        }
        for (int i = 0; i < channelsList.Length; ++i) {
            if (channelsList[i] > 15 || channelsList[i] < 0) {
                throw new IndexOutOfRangeException(
                    "List of channels contains incorrect channel number");
            }
        }
        FindExtremum(out dist, out azimuth, out channel, false, azimuthFrom,
                     azimuthTo, channelsList);
    }

    static double[] CalcXYZ(double dist, double azimuth, int channel) {
        double[] coord = new double[3];
        double w =
            VerticalAngles[channel] * Math.PI / 180; //Угол места в радианах
        double a = azimuth * Math.PI / 180; //Азимут в радианах
        coord[0] = dist * Math.Cos(w) * Math.Sin(a);
        coord[1] = dist * Math.Cos(w) * Math.Cos(a);
        coord[2] = dist * Math.Sin(w);
        return coord;
    }

    static int BestChannel(double w) {
        while (w < -180) {
            w += 360;
        }
        while (w >= 180) {
            w -= 360;
        }
        return IndexOfMinDistance(VerticalAngles, w);
    }

    static double GetVerticalAngle(int channel) {
        if (channel > 15 || channel < 0)
            throw new IndexOutOfRangeException(
                "Channel number is greater than 15 or less than 0");
        return VerticalAngles[channel];
    }

    void FindExtremum(out int dist, out double azimuth, out int channel,
                      bool findMax, double azimuthFrom, double azimuthTo,
                      int[] channelsList) {
        List<LidarDataBlock> LidarFormatedData =
            this.get_data(azimuthFrom, azimuthTo);
        if (LidarFormatedData.Count == 0) {
            throw new ArgumentOutOfRangeException(
                "Azimuth interval is too small");
        }
        LidarDataBlock block = LidarFormatedData.ElementAt(0);
        azimuth = azimuth = block.Azimuth;
        dist              = block.Distance1[channelsList[0]];
        channel           = channelsList[0];
        for (int i = 0; i < LidarFormatedData.Count; ++i) {
            block = LidarFormatedData.ElementAt(i);
            for (int j = 0; j < channelsList.Length; j++) {
                if (block.Distance1[channelsList[j]] > dist && findMax ||
                    block.Distance1[channelsList[j]] < dist && !findMax) {
                    azimuth = block.Azimuth;
                    dist    = block.Distance1[channelsList[j]];
                    channel = channelsList[j];
                }
            }
            for (int j = 0; j < block.Distance2.Length; j++) {
                if (block.Distance2[channelsList[j]] > dist && findMax ||
                    block.Distance2[channelsList[j]] < dist && !findMax) {
                    if (i < LidarFormatedData.Count - 1) {
                        LidarDataBlock nextBlock =
                            LidarFormatedData.ElementAt(i + 1);
                        double azimuth1 = block.Azimuth;
                        double azimuth2 = nextBlock.Azimuth;
                        if (azimuth2 < azimuth1) {
                            azimuth2 += 360;
                        }
                        azimuth = (block.Azimuth + nextBlock.Azimuth) / 2;
                    }
                    else {
                        LidarDataBlock firstBlock =
                            LidarFormatedData.ElementAt(0);
                        double deltaAzimuth = 0.5; //Шаг азимута
                        if (LidarFormatedData.Count > 1) {
                            LidarDataBlock secondBlock =
                                LidarFormatedData.ElementAt(1);
                            double azimuth1 = firstBlock.Azimuth;
                            double azimuth2 = secondBlock.Azimuth;
                            if (azimuth2 < azimuth1) {
                                azimuth2 += 360;
                            }
                            deltaAzimuth = (azimuth2 - azimuth1) / 2;
                        }
                        azimuth += deltaAzimuth;
                    }
                    dist    = block.Distance1[channelsList[j]];
                    channel = channelsList[j];
                    normalize_angle(ref azimuth);
                }
            }
        }
    }

    static int IndexOfMinDistance(double[] array, double x) {
        double minDif   = 999;
        int    minIndex = 0;
        for (int j = 0; j < array.Length; ++j) {
            double dif = Math.Abs(array[j] - x);
            if (j == 0 || dif < minDif) {
                minDif   = dif;
                minIndex = j;
            }
        }
        return minIndex;
    }
}
