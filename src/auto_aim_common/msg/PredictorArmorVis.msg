# PredictorArmorVis.msg

# Keep aligned with predictor::Armor::armor_status in PredictionType.hpp
int32 STATUS_NONEXIST=0
int32 STATUS_UNSEEN=1
int32 STATUS_AVAILABLE=2

int32 id
int32 status
geometry_msgs/Point center
float32 yaw
float32 theta
