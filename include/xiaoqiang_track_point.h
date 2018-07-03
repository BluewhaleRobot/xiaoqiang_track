#ifndef __XIAOQIANG_TRACK_POINT_H__
#define __XIAOQIANG_TRACK_POINT_H__

namespace XiaoqiangTrack
{
class Point
{
  public:
    float x;
    float y;
    Point(float px, float py)
    {
        x = px;
        y = py;
    }
};

} // namespace XiaoqiangTrack

#endif