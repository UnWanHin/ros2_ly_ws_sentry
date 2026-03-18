#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <tuple>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>
#include <TimeStamp/TimeStamp.hpp>
#include <Logger/Logger.hpp>

//// 在头文件里面使用using namespace std可能会导致冗余using ，但不是大问题
// TODO:
// 1.在实际使用过程中，将滤波改为时间间隔不定的情况应该会增强表现（反之则可能出现bug）**
// 2.匹配算法有小问题，匹配时先从检测到的点开始匹配，在转速过快情况下会出现错配。
//  应该改为将预测器点加入一起匹配。
// 3.可能需要稳定性判断，即程序发现波动过大自动重启等。
namespace ly_auto_aim::inline tracker {

struct Track {
    int id;
    cv::KalmanFilter kf;
    cv::Point2f lastPoint; // 滤波后的坐标，用于内部预测更新
    cv::Point2f rawPoint;  // 最近一次收到的原始测量坐标
    int livedFrames = 0;
    int missedFrames;
    bool _matched;

    Track(int id_, const cv::Point2f& pt, const cv::Point2f& vec_init = cv::Point2f(0,0)) : 
        id(id_), missedFrames(0), lastPoint(pt), rawPoint(pt), _matched(false) 
    {
        // 状态为 [x, y, vx, vy]
        kf = cv::KalmanFilter(4, 2, 0);
        // 状态转移矩阵 (dt假设为1)
        kf.transitionMatrix = (cv::Mat_<float>(4,4) << 
            1,0,1,0,
            0,1,0,1,
            0,0,1,0,
            0,0,0,1);
        // 观测矩阵：直接观测位置
        kf.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
        kf.measurementMatrix.at<float>(0,0) = 1.0f;
        kf.measurementMatrix.at<float>(1,1) = 1.0f;
        // 过程噪声
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        // 测量噪声
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        // 误差协方差矩阵
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
        // 初始状态：[x, y, 0, 0]
        kf.statePost = (cv::Mat_<float>(4,1) << pt.x, pt.y, vec_init.x, vec_init.y);
    }
};

class Matcher {
public:
    Matcher() : sameLabelError(70.0f), diffLabelError(70.0f), alpha(0.2f), autoFillMode(NO_FILL) {}

    using Point = cv::Point2f;

    template<typename T>
    std::map<int,const T*> track(const std::vector<std::pair<Point, const T*>>& currentPoints, 
                                Time::TimeStamp time, double timeRatio) 
    {
        double dt = 0.0;
        if(newTrack)
            newTrack = false;
        else
            // timeRatio is defined against a 20 ms reference interval.
            dt = (time - last_time).seconds() * 1000.0 / timeRatio;
        last_time = time;
        std::map<int, Point> output;
        std::vector<int> return_result(currentPoints.size(), -1);
        std::vector<bool> used(currentPoints.size(), false);

        // 计算自适应距离门限
        float adaptiveThreshold = sameLabelError * 0.6f + diffLabelError * 0.4f;
        // roslog::info("Adaptive Threshold: {}",adaptiveThreshold);
        // roslog::info("Same Label Error: {}",sameLabelError);
        // roslog::info("Diff Label Error: {}",diffLabelError);

        // 利用卡尔曼滤波预测所有轨迹的下一位置
        for (auto &trk : tracks) {
            // 更新状态转移矩阵，使用 dt 而不是固定的 1
            trk.kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
                1, 0, static_cast<float>(dt), 0,
                0, 1, 0, static_cast<float>(dt),
                0, 0, 1, 0,
                0, 0, 0, 1);

            // 更新过程噪声协方差矩阵
            // 对于状态 [x, y, vx, vy]，通常采用以下 Q 矩阵形式：
            // Q = sigma^2 * [ [dt^3/3,      0, dt^2/2,      0],
            //                 [     0, dt^3/3,      0, dt^2/2],
            //                 [dt^2/2,      0,      dt,      0],
            //                 [     0, dt^2/2,      0,     dt] ]
            float sigma = 1e-2f;
            float dt2 = static_cast<float>(dt * dt);
            float dt3 = static_cast<float>(dt2 * dt);
            trk.kf.processNoiseCov = (cv::Mat_<float>(4,4) <<
                sigma * dt3 / 3.0f, 0, sigma * dt2 / 2.0f, 0,
                0, sigma * dt3 / 3.0f, 0, sigma * dt2 / 2.0f,
                sigma * dt2 / 2.0f, 0, sigma * dt, 0,
                0, sigma * dt2 / 2.0f, 0, sigma * dt );

            // 更新测量噪声矩阵
            // 新增功能：根据 sameLabelError 实时更新测量噪声矩阵 R
            // 这里通过一个比例系数将 sameLabelError 映射为测量噪声，保证噪声下限不低于1e-1
            float r = std::max(1e-1f, static_cast<float>(sameLabelError * 0.5f));
            cv::setIdentity(trk.kf.measurementNoiseCov, cv::Scalar::all(r));
            cv::Mat prediction = trk.kf.predict();
            // 更新滤波坐标，用于匹配
            trk.lastPoint = cv::Point2f(prediction.at<float>(0), prediction.at<float>(1));
        }

        // 生成候选匹配对, std::tuple<float, int, int> 存储：距离、track索引、检测点索引
        std::vector<std::tuple<float, int, int>> candidateMatches;
        for (size_t i = 0; i < currentPoints.size(); i++) {
            cv::Point2f pt = currentPoints[i].first;
            for (size_t j = 0; j < tracks.size(); j++) {
                float d = cv::norm(pt - tracks[j].lastPoint);
                if (d < adaptiveThreshold) {
                    candidateMatches.emplace_back(d, static_cast<int>(j), static_cast<int>(i));
                }
            }
        }

        // 排序候选匹配, 按距离升序排列
        std::sort(candidateMatches.begin(), candidateMatches.end(), 
            [](const std::tuple<float, int, int>& a, const std::tuple<float, int, int>& b) {
                return std::get<0>(a) < std::get<0>(b);
            });

        std::vector<bool> detectedMatched(currentPoints.size(), false);
        std::vector<bool> trackInstanceMatched(tracks.size(), false);
        // 额外维护一个 set 来记录匹配过的 track.id（相同的 track.id 视为同一个对象）
        std::set<int> matchedTrackIds;

        // 处理匹配
        for (const auto &m : candidateMatches) {
            float d = std::get<0>(m);
            int trackIdx = std::get<1>(m);
            int detectIdx = std::get<2>(m);
            // 如果该检测点未匹配，且此 track 实例未匹配，同时该 track.id 也还未被匹配，则执行匹配
            if (!detectedMatched[detectIdx] &&
                !trackInstanceMatched[trackIdx] &&
                (matchedTrackIds.find(tracks[trackIdx].id) == matchedTrackIds.end())) 
            {
                detectedMatched[detectIdx] = true;
                trackInstanceMatched[trackIdx] = true;
                matchedTrackIds.insert(tracks[trackIdx].id);
                
                // 匹配成功，更新对应 Track
                Track &trk = tracks[trackIdx];
                cv::Mat measurement = (cv::Mat_<float>(2,1) << 
                    currentPoints[detectIdx].first.x, currentPoints[detectIdx].first.y);
                trk.kf.correct(measurement);
                trk.lastPoint = currentPoints[detectIdx].first;
                trk.rawPoint = currentPoints[detectIdx].first;
                trk.livedFrames++;
                trk.missedFrames = 0;
                output[trk.id] = currentPoints[detectIdx].first;
                return_result[detectIdx] = trk.id;
                used[detectIdx] = true;
                trk._matched = true;
                sameLabelError = alpha * d + (1 - alpha) * sameLabelError;
            }
        }

        // 处理未匹配轨迹 记录丢帧，并返回上一次的原始测量值
        for (auto it = tracks.begin(); it != tracks.end(); ) {
            if (!it->_matched) it->missedFrames++;
            it->_matched = false;
            
            if (it->missedFrames > maxMissedFrames) {
                // roslog::info("Track {} is lost",it->id);
                // roslog::info("Lived Frames: {}",it->livedFrames);
                averageLiveFrames = alpha * it->livedFrames + (1 - alpha) * averageLiveFrames;
                it = tracks.erase(it);
            } else {
                output[it->id] = it->lastPoint; // 统一使用NO_FILL模式   ，但是好像有点问题，记得对照来看
                ++it;
            }
        }

        // 处理新检测点 收集已有轨迹的编号及其x坐标，用于边界判断
        std::vector<std::pair<int, float>> currentOrder;
        for (const auto &p : output) {
            currentOrder.emplace_back(p.first, p.second.x);
        }
        std::sort(currentOrder.begin(), currentOrder.end(), 
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second < b.second;
            });

        std::set<int> usedIds;
        for (const auto &p : currentOrder) {
            usedIds.insert(p.first);
        }

        // 辅助函数：从 0～3 中选择一个不在 usedIds 中的编号
        auto selectAvailableId = [&usedIds]() -> int {
            for (int i = 0; i < 4; i++) {
                if (usedIds.find(i) == usedIds.end())
                    return i;
            }
            return -1; // 理论上不超过四个
        };

        // 添加新轨迹 对于未匹配到的检测点，新建轨迹时，按水平方向位置确定编号
        for (size_t i = 0; i < currentPoints.size(); i++) {
            if (!used[i]) {
                cv::Point2f pt = currentPoints[i].first;
                int newId = -1;
                if (currentOrder.empty()) {
                    newId = 0;
                } else {
                    float xMin = currentOrder.front().second;
                    float xMax = currentOrder.back().second;
                    int leftmostId = currentOrder.front().first;
                    int rightmostId = currentOrder.back().first;
                    float candidateError = 0.0f;
                    
                    if (pt.x < xMin) {
                        newId = (leftmostId + 3) % 4; // 左侧新点
                        candidateError = xMin - pt.x;
                    } else if (pt.x > xMax) {
                        newId = (rightmostId + 1) % 4; // 右侧新点
                        candidateError = pt.x - xMax;
                    } else {
                        newId = selectAvailableId();
                        if(newId == -1) newId = 0;
                    }
                    // 更新不同标号误差值：若候选误差大于零则更新
                    if (candidateError > 0)
                        diffLabelError = alpha * candidateError + (1 - alpha) * diffLabelError;
                }
                
                usedIds.insert(newId);
                {
                    cv::Point2f initVel(0, 0);
                    float maxSpeed = 0.0f;
                    // 遍历tracker寻找速度最大的tracker
                    // 使用最大加速初始收敛速度/使用最小可能导致不收敛
                    for (auto &trk : tracks) {
                        float vx = trk.kf.statePost.at<float>(2);
                        float vy = trk.kf.statePost.at<float>(3);
                        float speed = std::sqrt(vx*vx + vy*vy);
                        if (speed > maxSpeed) {
                            maxSpeed = speed;
                            initVel = cv::Point2f(vx, vy);
                        }
                    }
                
                    if(averageLiveFrames < minLivedFrames) {
                        initVel = cv::Point2f(0, 0);
                    }
                    
                    tracks.emplace_back(newId, pt, initVel);
                }
                output[newId] = pt; // 返回原始测量值
                return_result[i] = newId;
            }
        }
        
        // 构造返回结果
        std::map<int, const T*> output2;
        for (size_t i = 0; i < currentPoints.size(); i++) {
            output2[return_result[i]] = currentPoints[i].second;
        }
        return output2;
    }

    double getAverageLiveFrames() { return averageLiveFrames; }

private:
    bool newTrack = false;
    Time::TimeStamp last_time;
    std::vector<Track> tracks;
    const int maxMissedFrames = 8;
    const int minLivedFrames = 1; //lower than it means the track is not stable.
    // 自适应距离参数及平滑系数
    double sameLabelError;
    double diffLabelError;
    double averageLiveFrames = 10;
    const float alpha; // 平滑更新因子

    enum autoFillModeEnum { NO_FILL, LAST_DETECT, FILTER_PREDICT };
    const autoFillModeEnum autoFillMode = NO_FILL;
};

} // namespace ly_auto_aim::tracker
