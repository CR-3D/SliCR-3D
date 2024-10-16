// AdaptivePAInterpolator.cpp
// OrcaSlicer
//
// Implementation file for the AdaptivePAInterpolator class, providing methods to parse data and perform PA interpolation.

#include "AdaptivePAInterpolator.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <sstream>

/**
 * @brief Parses the input data and sets up the interpolators.
 * @param data A string containing the data in CSV format (PA, flow rate, acceleration).
 * @return 0 on success, -1 on error.
 */
int AdaptivePAInterpolator::parseAndSetData(const std::string& data) {
    flow_interpolators_.clear();
    accelerations_.clear();

    try {
        std::istringstream ss(data);
        std::string line;
        std::map<double, std::vector<std::pair<double, double>>> acc_to_flow_pa;

        while (std::getline(ss, line)) {
            std::istringstream lineStream(line);
            std::string value;
            double paValue, flowRate, acceleration;
            paValue = flowRate = acceleration = 0.f; // initialize all to zero.

            // Parse PA value
            if (std::getline(lineStream, value, ',')) {
                paValue = std::stod(value);
            }

            // Parse flow rate value
            if (std::getline(lineStream, value, ',')) {
                flowRate = std::stod(value);
            }

            // Parse acceleration value
            if (std::getline(lineStream, value, ',')) {
                acceleration = std::stod(value);
            }

            // Store the parsed values in a map with acceleration as the key
            acc_to_flow_pa[acceleration].emplace_back(flowRate, paValue);
        }

        // Iterate through the map to set up the interpolators
        for (const auto& kv : acc_to_flow_pa) {
            double acceleration = kv.first;
            const auto& data = kv.second;

            std::vector<double> flowRates;
            std::vector<double> paValues;

            for (const auto& pair : data) {
                flowRates.push_back(pair.first);
                paValues.push_back(pair.second);
            }

            // Only set up the interpolator if there are enough data points
            if (flowRates.size() > 1) {
                PchipInterpolatorHelper interpolator(flowRates, paValues);
                flow_interpolators_[acceleration] = interpolator;
                accelerations_.push_back(acceleration);
            }
        }
    } catch (const std::exception&) {
        m_isInitialised = false;
        return -1; // Error: Exception during parsing
    }
    m_isInitialised = true;
    return 0; // Success
}

double AdaptivePAInterpolator::interpolate_graph(double x_value) const {
    double y_value = 0.;
    if (this->data_size() < 1) {
        // nothing
    } else if (this->graph_points.size() == 1 || this->graph_points.front().x() >= x_value) {
        y_value = this->graph_points.front().y();
    } else if (this->graph_points.back().x() <= x_value) {
        y_value = this->graph_points.back().y();
    } else {
        // find first and second datapoint
        for (size_t idx = this->begin_idx; idx < this->end_idx; ++idx) {
            const auto &data_point = this->graph_points[idx];
            if (data_point.x() == x_value) {
                // lucky point
                y_value = data_point.y();
                break;
            } else if (data_point.x() < x_value) {
                // not yet, iterate
            } else if (idx == 0) {
                y_value = data_point.y();
                break;
            } else {
                // interpolate
                const auto &data_point_before = this->graph_points[idx - 1];
                assert(data_point.x() > data_point_before.x());
                assert(data_point_before.x() < x_value);
                assert(data_point.x() > x_value);
                if (this->type == GraphData::GraphType::SQUARE) {
                    y_value = data_point_before.y();
                } else if (this->type == GraphData::GraphType::LINEAR) {
                    const double interval     = data_point.x() - data_point_before.x();
                    const double ratio_before = (x_value - data_point_before.x()) / interval;
                    double mult = data_point_before.y() * (1 - ratio_before) + data_point.y() * ratio_before;
                    y_value = mult;
                } else if (this->type == GraphData::GraphType::SPLINE) {
                    // Cubic spline interpolation: see https://en.wikiversity.org/wiki/Cubic_Spline_Interpolation#Methods
                    const bool boundary_first_derivative = true; // true - first derivative is 0 at the leftmost and
                                                                 // rightmost point false - second ---- || -------
                    // TODO: cache (if the caller use my cache).
                    const int N = end_idx - begin_idx - 1; // last point can be accessed as N, we have N+1 total points
                    std::vector<float> diag(N + 1);
                    std::vector<float> mu(N + 1);
                    std::vector<float> lambda(N + 1);
                    std::vector<float> h(N + 1);
                    std::vector<float> rhs(N + 1);

                    // let's fill in inner equations
                    for (int i = 1 + begin_idx; i <= N + begin_idx; ++i) h[i] = this->graph_points[i].x() - this->graph_points[i - 1].x();
                    std::fill(diag.begin(), diag.end(), 2.f);
                    for (int i = 1 + begin_idx; i <= N + begin_idx - 1; ++i) {
                        mu[i]     = h[i] / (h[i] + h[i + 1]);
                        lambda[i] = 1.f - mu[i];
                        rhs[i]    = 6 * (float(this->graph_points[i + 1].y() - this->graph_points[i].y()) /
                                          (h[i + 1] * (this->graph_points[i + 1].x() - this->graph_points[i - 1].x())) -
                                      float(this->graph_points[i].y() - this->graph_points[i - 1].y()) /
                                          (h[i] * (this->graph_points[i + 1].x() - this->graph_points[i - 1].x())));
                    }

                    // now fill in the first and last equations, according to boundary conditions:
                    if (boundary_first_derivative) {
                        const float endpoints_derivative = 0;
                        lambda[0]                        = 1;
                        mu[N]                            = 1;
                        rhs[0] = (6.f / h[1]) * (float(this->graph_points[begin_idx].y() - this->graph_points[1 + begin_idx].y()) /
                                                     (this->graph_points[begin_idx].x() - this->graph_points[1 + begin_idx].x()) - endpoints_derivative);
                        rhs[N] = (6.f / h[N]) * (endpoints_derivative - float(this->graph_points[N + begin_idx - 1].y() - this->graph_points[N + begin_idx].y()) /
                                                                            (this->graph_points[N + begin_idx - 1].x() - this->graph_points[N + begin_idx].x()));
                    } else {
                        lambda[0] = 0;
                        mu[N]     = 0;
                        rhs[0]    = 0;
                        rhs[N]    = 0;
                    }

                    // the trilinear system is ready to be solved:
                    for (int i = 1; i <= N; ++i) {
                        float multiple = mu[i] / diag[i - 1]; // let's subtract proper multiple of above equation
                        diag[i] -= multiple * lambda[i - 1];
                        rhs[i] -= multiple * rhs[i - 1];
                    }
                    // now the back substitution (vector mu contains invalid values from now on):
                    rhs[N] = rhs[N] / diag[N];
                    for (int i = N - 1; i >= 0; --i) rhs[i] = (rhs[i] - lambda[i] * rhs[i + 1]) / diag[i];

                    //now interpolate at our point
                    size_t curr_idx = idx - begin_idx;
                    y_value = (rhs[curr_idx - 1] * pow(this->graph_points[idx].x() - x_value, 3) +
                            rhs[curr_idx] * pow(x_value - this->graph_points[idx - 1].x(), 3)) /
                            (6 * h[curr_idx]) +
                        (this->graph_points[idx - 1].y() - rhs[curr_idx - 1] * h[curr_idx] * h[curr_idx] / 6.f) *
                            (this->graph_points[idx].x() - x_value) / h[curr_idx] +
                        (this->graph_points[idx].y() - rhs[curr_idx] * h[curr_idx] * h[curr_idx] / 6.f) *
                            (x_value - this->graph_points[idx - 1].x()) / h[curr_idx];
                } else {
                    assert(false);
                }
                break;
            }
        }
    }
    return y_value;
}



/**
 * @brief Interpolates the PA value for the given flow rate and acceleration.
 * @param flow_rate The flow rate at which to interpolate.
 * @param acceleration The acceleration at which to interpolate.
 * @return The interpolated PA value, or -1 if interpolation fails.
 */
double AdaptivePAInterpolator::operator()(double flow_rate, double acceleration) {
    std::vector<double> pa_values;
    std::vector<double> acc_values;

    // Estimate PA value for every flow to PA model for the given flow rate
    for (const auto& kv : flow_interpolators_) {
        double pa_value = kv.second.interpolate(flow_rate);
        
        // Check if the interpolated PA value is valid
        if (pa_value != -1) {
            pa_values.push_back(pa_value);
            acc_values.push_back(kv.first);
        }
    }

    // Check if there are enough acceleration values for interpolation
    if (acc_values.size() < 2) {
        // Special case: Only one acceleration value
        if (acc_values.size() == 1) {
            return std::round(pa_values[0] * 1000.0) / 1000.0; // Rounded to 3 decimal places
        }
        return -1; // Error: Not enough data points for interpolation
    }

    // Create a new PchipInterpolatorHelper for PA-acceleration interpolation
    // Use the estimated PA values from the for loop above and their corresponding accelerations to
    // generate the new PCHIP model. Then run this model to interpolate the PA value for the given acceleration value.
    PchipInterpolatorHelper pa_accel_interpolator(acc_values, pa_values);
    return std::round(pa_accel_interpolator.interpolate(acceleration) * 1000.0) / 1000.0; // Rounded to 3 decimal places
}
