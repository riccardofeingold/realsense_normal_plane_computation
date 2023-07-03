#pragma once
#include <vector>
#include <mutex>
#include <array>
#include <librealsense2/rs.hpp>
#include <Eigen/Eigenvalues> 


namespace rs2
{
    namespace depth_quality
    {
        struct float3
        {
            float x, y, z;

            float length() const { return sqrt( x * x + y * y + z * z ); }

            float3 normalize() const { return ( length() > 0 ) ? float3{ x / length(), y / length(), z / length() } : *this; }
        };

        inline float3 operator*( const float3 & a, float t )
        {
            return { a.x * t, a.y * t, a.z * t };
        }

        inline float3 operator/( const float3 & a, float t )
        {
            return { a.x / t, a.y / t, a.z / t };
        }

        inline float3 operator+( const float3 & a, const float3 & b )
        {
            return { a.x + b.x, a.y + b.y, a.z + b.z };
        }

        inline float3 operator-( const float3 & a, const float3 & b )
        {
            return { a.x - b.x, a.y - b.y, a.z - b.z };
        }
        struct plane
        {
            double a;
            double b;
            double c;
            double d;
        };

        inline plane plane_from_point_and_normal(const float3& point, const float3& normal)
        {
            return{ normal.x, normal.y, normal.z, -(normal.x*point.x + normal.y*point.y + normal.z*point.z) };
        }

        // Based on: calculating the eigenvectors and choosing the smallest one => normal
        // http://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
        inline plane plane_from_points(const std::vector<float3> points)
        {
            float3 error = {1,1,1};
            if (points.size() < 3) return plane_from_point_and_normal(error, error.normalize());

            float3 sum = { 0,0,0 };
            for (auto point : points) sum = sum + point;

            float3 centroid = sum / float(points.size());

            double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
            for (auto point : points) {
                float3 temp = point - centroid;
                xx += temp.x * temp.x;
                xy += temp.x * temp.y;
                xz += temp.x * temp.z;
                yy += temp.y * temp.y;
                yz += temp.y * temp.z;
                zz += temp.z * temp.z;
            }

            // calculating eigenvectors and eigenvalues
            Eigen::Matrix3d covariance_matrix;
            covariance_matrix(0, 0) = xx;
            covariance_matrix(0, 1) = xy;
            covariance_matrix(0, 2) = xz;
            covariance_matrix(1, 0) = xy;
            covariance_matrix(1, 1) = yy;
            covariance_matrix(1, 2) = yz;
            covariance_matrix(2, 0) = xz;
            covariance_matrix(2, 1) = yz;
            covariance_matrix(2, 2) = zz;


            // initialize solver
            Eigen::EigenSolver<Eigen::Matrix3d> es(covariance_matrix);
            
            Eigen::VectorXcd ev = es.eigenvalues();
            
            std::complex<double> smallest_x_ev = ev(0);
            int index = 0;
            for (size_t i = 1; i < ev.size(); ++i)
            {
                if (smallest_x_ev.real() > ev(i).real())
                {
                    smallest_x_ev = ev(i);
                    index = i;
                }
            }

            Eigen::Vector3d eigenvector = es.eigenvectors().col(index).real();
            float3 dir = {};
            if (eigenvector(2) < 0)
                dir = {static_cast<float>(-eigenvector(0)), static_cast<float>(-eigenvector(1)), static_cast<float>(-eigenvector(2))};
            else
                dir = {static_cast<float>(eigenvector(0)), static_cast<float>(eigenvector(1)), static_cast<float>(eigenvector(2))};

            return plane_from_point_and_normal(centroid, dir.normalize());
        }
        // inline plane plane_from_points(const std::vector<float3> points)
        // {
        //     float3 error = {1,1,1};
        //     if (points.size() < 3) return plane_from_point_and_normal(error, error.normalize());

        //     float3 sum = { 0,0,0 };
        //     for (auto point : points) sum = sum + point;

        //     float3 centroid = sum / float(points.size());

        //     double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        //     for (auto point : points) {
        //         float3 temp = point - centroid;
        //         xx += temp.x * temp.x;
        //         xy += temp.x * temp.y;
        //         xz += temp.x * temp.z;
        //         yy += temp.y * temp.y;
        //         yz += temp.y * temp.z;
        //         zz += temp.z * temp.z;
        //     }

        //     double det_x = yy*zz - yz*yz;
        //     double det_y = xx*zz - xz*xz;
        //     double det_z = xx*yy - xy*xy;

        //     double det_max = std::max({ det_x, det_y, det_z });
        //     if (det_max <= 0) return{ 0, 0, 0, 0 };

        //     float3 dir{};
        //     if (det_max == det_x)
        //     {
        //         float a = static_cast<float>((xz*yz - xy*zz) / det_x);
        //         float b = static_cast<float>((xy*yz - xz*yy) / det_x);
        //         dir = { 1, a, b };
        //     }
        //     else if (det_max == det_y)
        //     {
        //         float a = static_cast<float>((yz*xz - xy*zz) / det_y);
        //         float b = static_cast<float>((xy*xz - yz*xx) / det_y);
        //         dir = { a, 1, b };
        //     }
        //     else
        //     {
        //         float a = static_cast<float>((yz*xy - xz*yy) / det_z);
        //         float b = static_cast<float>((xz*xy - yz*xx) / det_z);
        //         dir = { a, b, 1 };
        //     }

        //     return plane_from_point_and_normal(centroid, dir.normalize());
        // }
    }
}