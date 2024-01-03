#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <OpenCascade/OpenCascade.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <gp_Pnt.hxx>

// B-spline曲線を生成し、autoware_auto_planning_msgs::msg::Trajectory に変換する関数
autoware_auto_planning_msgs::msg::Trajectory generateBSplineTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory& input_trajectory)
{
    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    trajectory.header.stamp = rclcpp::Clock().now();
    trajectory.header.frame_id = "map";  // フレームIDは必要に応じて変更

    std::vector<gp_Pnt> trajectoryPoints;

    // 入力のTrajectoryから点を取り出してOpenCASCADE用のデータ構造に変換
    for (const auto& point : input_trajectory.points) {
        gp_Pnt pnt(point.pose.position.x, point.pose.position.y, point.pose.position.z);
        trajectoryPoints.push_back(pnt);
    }

    Standard_Integer numPoints = static_cast<Standard_Integer>(trajectoryPoints.size());
    TColgp_Array1OfPnt pointArray(1, numPoints);

    for (Standard_Integer i = 1; i <= numPoints; ++i) {
        pointArray.SetValue(i, trajectoryPoints[i - 1]);
    }

    TColStd_Array1OfReal paramArray(1, numPoints);

    for (Standard_Integer i = 1; i <= numPoints; ++i) {
        paramArray.SetValue(i, i - 1);
    }

    GeomAPI_Interpolate interpolator(pointArray, Standard_False, 1.0e-6);
    interpolator.Perform(paramArray);

    Handle(Geom_BSplineCurve) bsplineCurve = interpolator.Curve();

    // B-spline曲線上の点を生成し、autoware_auto_planning_msgs::msg::Trajectory に追加
    for (Standard_Real u = bsplineCurve->FirstParameter(); u <= bsplineCurve->LastParameter(); u += 0.1) {
        gp_Pnt point = bsplineCurve->Value(u);

        autoware_auto_planning_msgs::msg::TrajectoryPoint trajectoryPoint;
        trajectoryPoint.pose.position.x = point.X();
        trajectoryPoint.pose.position.y = point.Y();
        trajectoryPoint.pose.position.z = point.Z();

        trajectory.points.push_back(trajectoryPoint);
    }

    return trajectory;
}

int main() {
    // 本当の点で input_trajectory を埋める
    autoware_auto_planning_msgs::msg::Trajectory input_trajectory;
    // ...

    // B-spline曲線を生成し、autoware_auto_planning_msgs::msg::Trajectory に変換
    autoware_auto_planning_msgs::msg::Trajectory bsplineTrajectory = generateBSplineTrajectory(input_trajectory);

    // 'bsplineTrajectory' にはB-spline曲線上のポイントが含まれる

    return 0;
}
