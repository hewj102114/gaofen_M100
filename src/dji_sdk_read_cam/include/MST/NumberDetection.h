#ifndef NUMBERDETECTION_H
#define NUMBERDETECTION_H

void Calcu_attitude(Point2f pnt_tl_src, Point2f pnt_tr_src, Point2f pnt_br_src, Point2f pnt_bl_src);
int Num_detection(cv::Mat &img, dji_sdk::Reldist & result);
#endif
