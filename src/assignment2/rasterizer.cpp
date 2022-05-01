// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>

#define MSAA true


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


auto crossProduct(Vector3f v1, Vector3f v2){

    auto result = v1.cross(v2);
    return result;
}

static bool sameSide(Vector3f p1, Vector3f p2, Vector3f a, Vector3f b) {

	auto cp1 = crossProduct(b - a, p1 - a);
	auto cp2 = crossProduct(b - a, p2 - a);

	if (cp1.dot(cp2) > 0.0f)
		return true;
	else
		return false;

}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Vector3f P = Vector3f(x, y, _v[0].z());
	Vector3f AC = _v[2] - _v[0];
	Vector3f CB = _v[1] - _v[2];
	Vector3f BA = _v[0] - _v[1];
	Vector3f AP = P - _v[0];
	Vector3f BP = P - _v[1];
	Vector3f CP = P - _v[2];

	//if cross product in the same direction ,its inside the triangle
	if (AP.cross(AC).dot(BP.cross(BA)) > 0.0f &&
		BP.cross(BA).dot(CP.cross(CB)) > 0.0f &&
		CP.cross(CB).dot(AP.cross(AC)) > 0.0f)
	{
		return true;
	}
	return false;
}

//static bool insideTriangle(int x, int y, const Vector3f* _v)
//{   
//    Vector3f p(x+0.5f, y+0.5f, 1.0f);
//
//	//Vector3f& A = _v[0];
//	//Vector3f& B = _v[1];
//	//Vector3f& C = _v[2];
//
//	//Vector3f AB = B - A;
//	//Vector3f BC = C - B;
//	//Vector3f CA = A - C;
//
//	//Vector3f AP = P - A;
//	//Vector3f BP = P - B;
//	//Vector3f CP = P - C;
//
//     auto a = _v[0];
//     auto b = _v[1];
//     auto c = _v[2];
//
//    if (sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b))
//        return true;
//    else
//        return false;
//    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
//}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

float maxOfThreeNumbers(float a, float b, float c)
{
    float temp = std::max(a, b);
    float temp1 = std::max(temp, c);
    return temp1;

}


float minOfThreeNumbers(float a, float b, float c)
{
    float temp = std::min(a, b);
    float temp1 = std::min(temp, c);
    return temp1;

}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    auto max_x = maxOfThreeNumbers(t.v[0].x(), t.v[1].x(), t.v[2].x());
    auto min_x = minOfThreeNumbers(t.v[0].x(), t.v[1].x(), t.v[2].x());
    auto max_y = maxOfThreeNumbers(t.v[0].y(), t.v[1].y(), t.v[2].y());
    auto min_y = minOfThreeNumbers(t.v[0].y(), t.v[1].y(), t.v[2].y());

    std::vector<Vector3f> samples =
    {
        {0.25f, 0.25f, 0.0f},
        {0.25f, 0.75f, 0.0f},
        {0.75f, 0.25f, 0.0f},
        {0.75f, 0.75f, 0.0f},
    };

    for(int x=min_x; x<max_x; x++){
        for(int y=min_y; y<max_y; y++){
            if (MSAA)
            {
                int sample_count = 0;
                int sample_times = 0;
                int drawable = 0;
                for (auto& _sample : samples)
                {
                    float x_sample = x + _sample.x();
                    float y_sample = y + _sample.y();

					bool state = insideTriangle(x_sample, y_sample, t.v);
                    if (state) {
                        auto [alpha, beta, gamma] = computeBarycentric2D(x_sample, y_sample, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        int buf_index = get_index(x, y);
                        if (z_interpolated < multi_sample_buf[buf_index][sample_count])
                        {
                            multi_sample_buf[buf_index][sample_count] = z_interpolated;
                            drawable++;
                        }
                        sample_times ++;

                    }
                    sample_count ++;
                }

                if (drawable)
                {
					Vector3f color = t.getColor() * sample_times / 4.0f;
					set_pixel(Vector3f(x, y, 0), color);
                }
            }
            else
            {
				bool state = insideTriangle(x, y, t.v);
				if (state) {
					auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;

					int buf_index = get_index(x, y);
					if (z_interpolated >= depth_buf[buf_index]) continue;

					depth_buf[buf_index] = z_interpolated;

					Vector3f color = t.getColor();
					set_pixel(Vector3f(x, y, 0), color);
				}
            }
        }
    }

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    // auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // z_interpolated *= w_reciprocal;


    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::vector<float> _inf(4, std::numeric_limits<float>::infinity());
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(multi_sample_buf.begin(), multi_sample_buf.end(), _inf);
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    multi_sample_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on


