#include"utils/utils.h"
using namespace autopilot::utils;

// Point3d
bool Point3D::operator==(const Point3D& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0)) < 0.001;
}

double Point3D::Distance(const Point3D& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0));
}

boost::shared_ptr<carla::client::Waypoint> WayPoint_Node::GetWayPoint() const{
  return waypoint_;
}

double WayPoint_Node::GetDistance() const{
  return distance_;
}

void UpdateVehicleState(VehicleState* m_state){
  
}

bool ComputePathProfile(const std::vector<carla::geom::Location> route_wypoint,
                        std::vector<double>* headings,
                        std::vector<double>* accumulated_s,
                        std::vector<double>* kappas,
                        std::vector<double>* dkappas) 
{
    headings->clear();
    kappas->clear();
    dkappas->clear();

    if (route_wypoint.size() < 2) 
    {
      std::cout<<"start position and target position are too close"<<std::endl;
        return false;
    }
    std::vector<double> dxs;
    std::vector<double> dys;
    std::vector<double> y_over_s_first_derivatives;
    std::vector<double> x_over_s_first_derivatives;
    std::vector<double> y_over_s_second_derivatives;
    std::vector<double> x_over_s_second_derivatives;

    // Get finite difference approximated dx and dy for heading and kappa calculation
    std::size_t points_size = route_wypoint.size();
    for (std::size_t i = 0; i < points_size; ++i) 
    {
        double x_delta = 0.0;
        double y_delta = 0.0;
        if (i == 0) 
        {
            x_delta = (route_wypoint[i + 1].x - route_wypoint[i].x);
            y_delta = (route_wypoint[i + 1].y - route_wypoint[i].y);
        } 
        else if (i == points_size - 1) 
        {
            x_delta = (route_wypoint[i].x - route_wypoint[i - 1].x);
            y_delta = (route_wypoint[i].y - route_wypoint[i - 1].y);
        } 
        else 
        {
            x_delta = 0.5 * (route_wypoint[i + 1].x - route_wypoint[i - 1].x);
            y_delta = 0.5 * (route_wypoint[i + 1].y - route_wypoint[i - 1].y);
        }
        dxs.push_back(x_delta);
        dys.push_back(y_delta);
    }

    // Heading calculation
    for (std::size_t i = 0; i < points_size; ++i) 
    {
        headings->push_back(std::atan2(dys[i], dxs[i]));
    }

    // Get linear interpolated s for dkappa calculation
    double distance = 0.0;
    accumulated_s->push_back(distance);
    double fx = route_wypoint[0].x;
    double fy = route_wypoint[0].y;
    double nx = 0.0;
    double ny = 0.0;
    for (std::size_t i = 1; i < points_size; ++i) 
    {
        nx = route_wypoint[i].x;
        ny = route_wypoint[i].y;
        double end_segment_s = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
        accumulated_s->push_back(end_segment_s + distance);
        distance += end_segment_s;
        fx = nx;
        fy = ny;
    }

    // Get finite difference approximated first derivative of y and x respective
    // to s for kappa calculation
    for (std::size_t i = 0; i < points_size; ++i) 
    {
        double xds = 0.0;
        double yds = 0.0;
        if (i == 0) 
        {
            xds = (route_wypoint[i + 1].x - route_wypoint[i].x) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
            yds = (route_wypoint[i + 1].y - route_wypoint[i].y) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
        } 
        else if (i == points_size - 1) 
        {
            xds = (route_wypoint[i].x - route_wypoint[i - 1].x) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
            yds = (route_wypoint[i].y - route_wypoint[i - 1].y) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
        } 
        else 
        {
            xds = (route_wypoint[i + 1].x - route_wypoint[i - 1].x) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            yds = (route_wypoint[i + 1].y - route_wypoint[i - 1].y) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        }
        x_over_s_first_derivatives.push_back(xds);
        y_over_s_first_derivatives.push_back(yds);
    }

    // Get finite difference approximated second derivative of y and x
    // respective to s for kappa calculation
    for (std::size_t i = 0; i < points_size; ++i) 
    {
        double xdds = 0.0;
        double ydds = 0.0;
        if (i == 0) 
        {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
        } 
        else if (i == points_size - 1) 
        {
            xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
            ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
        } 
        else 
        {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        }
        x_over_s_second_derivatives.push_back(xdds);
        y_over_s_second_derivatives.push_back(ydds);
    }

    for (std::size_t i = 0; i < points_size; ++i) 
    {
        double xds = x_over_s_first_derivatives[i];
        double yds = y_over_s_first_derivatives[i];
        double xdds = x_over_s_second_derivatives[i];
        double ydds = y_over_s_second_derivatives[i];
        double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        kappas->push_back(kappa);
    }

    // Dkappa calculation
    for (std::size_t i = 0; i < points_size; ++i) 
    {
        double dkappa = 0.0;
        if (i == 0) 
        {
            dkappa = (kappas->at(i + 1) - kappas->at(i)) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
        } 
        else if (i == points_size - 1) 
        {
            dkappa = (kappas->at(i) - kappas->at(i - 1)) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
        } 
        else 
        {
            dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        }
        dkappas->push_back(dkappa);
    }
    return true;
}