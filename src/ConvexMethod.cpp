#include "obj_planner/ConvexMethod.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace {

constexpr double pi{3.14159265};
constexpr double epsilon{1e-3};

/// Wrap angle (radians) between [0, 2pi)
double wrap_to_2pi(double angle) {
    angle = fmod(angle, 2.0 * pi);
    if (angle < 0.0) angle += 2.0 * pi;
    return angle;
}

/// Only let number reside in [0, 1]
void clamp_between_0_and_1(double& number) {
    if (number < 0.0) {
        number = 0.0;
    } else if (number > 1.0) {
        number = 1.0;
    }
}

}  // namespace

// Central classification fn
std::optional<LeftRightResults> ConvexMethod::classify(const geometry_msgs::msg::PoseArray& detections_array) {
    std::optional<LeftRightResults> classification{std::nullopt};
    auto detections_2d = get_detections_vector(detections_array);

    // Remove false double detections
    detections_2d = this->remove_duplicate_detections(detections_2d);

    auto convex_hull = get_convex_hull(detections_2d);

    if (is_convex_hull_valid(convex_hull)) {
        // Determine detection strategy to dispatch
        const auto [scenario, unit] = determine_scenario(detections_2d);

        if (scenario == Scenario::kStraight) {
            scenario_classifier = straight_scenario_classifier;
        } else if (scenario == Scenario::kLeft) {
            scenario_classifier = left_scenario_classifier;
        } else {
            std::reverse(std::begin(convex_hull), std::end(convex_hull));  // Make hull clock-wise
            scenario_classifier = right_scenario_classifier;
        }

        // Classify detections into left and right using strategy
        classification = scenario_classifier->classify(convex_hull, detections_2d);

        // Return None if pairs to one side only
        if (classification->left_detections.empty() || classification->right_detections.empty()) {
            if (this->logger) {
                RCLCPP_INFO(*this->logger, "Classifier only found one side!");
            }

            return std::nullopt;
        } else {
            // Visualization
            if (this->params.debug) {
                ConvexMethod::visualize_hull(classification, convex_hull, scenario, unit);
            }
        }
    }

    return classification;
}

void ConvexMethod::visualize_hull(std::optional<LeftRightResults>& classification,
                                  std::vector<cv::Point2d>& convex_hull, Scenario scenario, const cv::Vec2f& line) {
    if (!convex_hull.empty()) {
        cv::Mat drawing = cv::Mat::zeros(cv::Size(1500, 1500), CV_8UC3);

        auto convert_to_image_space = [&drawing](const cv::Point2d& pt) {
            cv::Point new_pt{};

            // Kart view is bottom middle of image
            auto half_w = drawing.size[0] / 2;
            new_pt.x = (int(pt.y * 100) * -1 + half_w);
            new_pt.y = (int(pt.x * 100) * -1 + drawing.size[1]);
            return new_pt;
        };

        // Convert from meters in float to cm in int to work on a pixel level
        std::vector<cv::Point> draw_pts{};
        std::transform(convex_hull.begin(), convex_hull.end(), std::back_inserter(draw_pts), convert_to_image_space);

        // Draw hull
        drawContours(drawing, std::vector<std::vector<cv::Point>>{{draw_pts}}, 0, cv::Scalar{255, 255, 255}, 3);

        if (classification) {
            // Draw classifications
            for (auto& pt : classification.value().left_detections) {
                cv::circle(drawing, convert_to_image_space(pt), 10, cv::Scalar{0, 255, 0}, 3);
            }

            for (auto& pt : classification.value().right_detections) {
                cv::circle(drawing, convert_to_image_space(pt), 10, cv::Scalar{255, 0, 0}, 4);
            }
        }

        // Draw line of best fit
        cv::line(drawing, convert_to_image_space(cv::Point2d{}), convert_to_image_space(cv::Point2d{1000 * line}),
                 cv::Scalar{255, 255, 255});

        // Add helper text
        cv::putText(drawing, "Blue = Right", cv::Point{10, 70}, cv::FONT_HERSHEY_COMPLEX, 3, cv::Scalar{255, 0, 0}, 5);
        cv::putText(drawing, "Green = Left", cv::Point{10, 150}, cv::FONT_HERSHEY_COMPLEX, 3, cv::Scalar{0, 255, 0}, 5);

        switch (scenario) {
            case Scenario::kStraight:
                cv::putText(drawing, "Scenario = Straight", cv::Point{10, 215}, cv::FONT_HERSHEY_COMPLEX, 3,
                            cv::Scalar{0, 0, 255}, 5);
                break;
            case Scenario::kLeft:
                cv::putText(drawing, "Scenario = Left", cv::Point{10, 215}, cv::FONT_HERSHEY_COMPLEX, 3,
                            cv::Scalar{0, 255, 0}, 5);
                break;
            case Scenario::kRight:
                cv::putText(drawing, "Scenario = Right", cv::Point{10, 215}, cv::FONT_HERSHEY_COMPLEX, 3,
                            cv::Scalar{255, 0, 0}, 5);
                break;
        }

        cv::resize(drawing, drawing, cv::Size{512, 512});

        cv::imshow("Hull", drawing);
        cv::pollKey();
    }
}

std::vector<cv::Point2d> ConvexMethod::get_detections_vector(const geometry_msgs::msg::PoseArray& detections_array) {
    std::vector<cv::Point2d> detections_2d{};
    for (const auto& pose : detections_array.poses) {
        detections_2d.emplace_back(pose.position.x, pose.position.y);
    }
    return detections_2d;
}

std::vector<cv::Point2d> ConvexMethod::get_convex_hull(const std::vector<cv::Point2d>& detections_2d) {
    std::vector<cv::Point2d> convex_hull{};
    if (detections_2d.size() >= 3U) {
        // The convex hull function only works on floats, so do a type dance
        std::vector<cv::Point2f> convex_hull_float{};
        std::vector<cv::Point2f> detections_float(std::cbegin(detections_2d), std::cend(detections_2d));

        cv::convexHull(detections_float, convex_hull_float);
        std::transform(std::cbegin(convex_hull_float), std::cend(convex_hull_float), std::back_inserter(convex_hull),
                       [](const auto& point) {
                           return cv::Point2d{static_cast<double>(point.x), static_cast<double>(point.y)};
                       });
    }

    return convex_hull;
}

bool ConvexMethod::is_convex_hull_valid(const std::vector<cv::Point2d>& convex_hull) {
    if (convex_hull.size() < 3U) {
        return false;
    }

    std::vector<cv::Point2f> fhull{convex_hull.begin(), convex_hull.end()};

    // Hull is valid if it is larger than some threshold
    double area = cv::contourArea(fhull);

    return area > params.convex_hull_area_threshold;
}

std::tuple<Scenario, cv::Vec2f> ConvexMethod::determine_scenario(const std::vector<cv::Point2d>& detections_2d) {
    cv::Mat line{};

    // RANSAC a line of best fit
    cv::fitLine(detections_2d, line, cv::DIST_L2, 0, 0.01, 0.01);

    // Fit returns a unit vector along line
    auto vx = line.at<float>(0);
    auto vy = line.at<float>(1);
    cv::Vec2f unit{vx, vy};
    cv::Vec2f y_axis{0, 1};
    float res = y_axis.dot(unit);

    // Angle between line and y-axis, left facing
    float y_angle = std::acos(res);
    // Convert to deg
    y_angle = y_angle * 180.0f / M_PIf;

    // Determine scenario from angle
    Scenario scenario;
    if (y_angle == std::clamp(y_angle, 90.0f - this->params.turn_threshold, 90.0f + this->params.turn_threshold)) {
        scenario = Scenario::kStraight;
    } else if (y_angle > 90) {
        scenario = Scenario::kRight;
    } else if (y_angle < 90) {
        scenario = Scenario::kLeft;
    }

    return std::make_tuple(scenario, unit);
}

std::vector<cv::Point2d> ConvexMethod::remove_duplicate_detections(const std::vector<cv::Point2d>& detections_2d) {
    std::vector<size_t> skip_list{};

    // Find all points that are too close to other points
    for (size_t i = 0; i < detections_2d.size(); i++) {
        auto pt1 = detections_2d[i];
        std::vector<double> distance;

        for (size_t j = 0; j < detections_2d.size(); j++) {
            // Skip skipped points
            if (std::find(skip_list.begin(), skip_list.end(), j) != skip_list.end() || j == i) {
                continue;
            }

            auto pt2 = detections_2d[j];
            distance.push_back(std::hypot(pt1.x - pt2.x, pt1.y - pt2.y));
        }

        // If any other cone is too close, remove
        for (const auto& item : distance) {
            if (item < 0.5f) {
                skip_list.push_back(i);
                break;
            }
        }
    }

    std::vector<cv::Point2d> out{};
    for (size_t i = 0; i < detections_2d.size(); ++i) {
        if (std::find(skip_list.begin(), skip_list.end(), i) != skip_list.end()) {
            continue;
        }

        out.push_back(detections_2d[i]);
    }

    return out;
}

LeftRightResults StraightScenarioClassifier::classify([[maybe_unused]] std::vector<cv::Point2d>& convex_hull,
                                                      const std::vector<cv::Point2d>& detections_2d) {
    LeftRightResults classification{};

    // Simply classify by being left or right of kart
    for (const cv::Point2d& detection : detections_2d) {
        if (detection.y > 0.0) {
            classification.left_detections.push_back(detection);
        } else if (detection.y <= 0.0) {
            classification.right_detections.push_back(detection);
        }
    }

    return classification;
}

std::size_t LeftScenarioClassifier::find_start_idx_on_convex_hull(const std::vector<double>& convex_hull_angles) {
    const auto start_it = std::min_element(std::cbegin(convex_hull_angles), std::cend(convex_hull_angles));
    return static_cast<std::size_t>(std::distance(std::cbegin(convex_hull_angles), start_it));
}

std::vector<cv::Point2d>& LeftScenarioClassifier::get_outside_detections_result(LeftRightResults& classification) {
    return classification.right_detections;
}

std::vector<cv::Point2d>& LeftScenarioClassifier::get_inside_detections_result(LeftRightResults& classification) {
    return classification.left_detections;
}

LeftRightResults LeftScenarioClassifier::classify(std::vector<cv::Point2d>& convex_hull,
                                                  const std::vector<cv::Point2d>& detections_2d) {
    auto parent_res = TurningScenarioClassifier::classify(convex_hull, detections_2d);

    // Bump to the left for more aggressive turns
    for (auto& item : parent_res.left_detections) {
        item.y += 2.0;  //TODO see if required
    }

    return parent_res;
}

std::size_t RightScenarioClassifier::find_start_idx_on_convex_hull(const std::vector<double>& convex_hull_angles) {
    const auto start_it = std::max_element(std::cbegin(convex_hull_angles), std::cend(convex_hull_angles));
    return static_cast<std::size_t>(std::distance(std::cbegin(convex_hull_angles), start_it));
}

std::vector<cv::Point2d>& RightScenarioClassifier::get_outside_detections_result(LeftRightResults& classification) {
    return classification.left_detections;
}

std::vector<cv::Point2d>& RightScenarioClassifier::get_inside_detections_result(LeftRightResults& classification) {
    return classification.right_detections;
}

LeftRightResults RightScenarioClassifier::classify(std::vector<cv::Point2d>& convex_hull,
                                                   const std::vector<cv::Point2d>& detections_2d) {
    auto parent_res = TurningScenarioClassifier::classify(convex_hull, detections_2d);

    // Bump to the left for more aggressive turns
    for (auto& item : parent_res.right_detections) {
        item.y -= 2.0;  //TODO see if required
    }

    return parent_res;
}

LeftRightResults TurningScenarioClassifier::classify(std::vector<cv::Point2d>& convex_hull,
                                                     const std::vector<cv::Point2d>& detections_2d) {
    LeftRightResults classification{};

    // Calculate angles relative to (-1, 0) direction
    std::vector<double> convex_hull_angles{};
    std::transform(std::cbegin(convex_hull), std::cend(convex_hull), std::back_inserter(convex_hull_angles),
                   [](const auto& point_2d) {
                       double angle_from_behind{3.0 / 2.0 * pi - std::atan2(point_2d.x, point_2d.y)};
                       return wrap_to_2pi(angle_from_behind);
                   });
    const std::size_t start_idx = find_start_idx_on_convex_hull(convex_hull_angles);

    // Reorder the convexhull to have start_idx be shifted to be located at index 0.
    std::rotate(std::begin(convex_hull), std::next(std::begin(convex_hull), start_idx), std::end(convex_hull));

    // Find the end segment idx on the convex hull
    // TODO: make sure the hull iteration doesn't need to be wrapped around
    const auto end_segment_idx = find_end_segment_index(convex_hull);

    // Add points along hull to outside
    std::vector<cv::Point2d>& outside_detections_result = get_outside_detections_result(classification);
    std::transform(std::cbegin(convex_hull), std::next(std::cbegin(convex_hull), end_segment_idx + 1U),
                   std::back_inserter(outside_detections_result), [](const auto& point) {
                       return cv::Point2d{point.x, point.y};
                   });

    // Find unused points
    auto unused_detections = find_unused_detections(detections_2d, outside_detections_result);

    // Try to associate unused detections to outside
    associate_unused_detections(outside_detections_result, unused_detections);

    // All unused after association are on the inside
    std::vector<cv::Point2d>& inside_detections_result = get_inside_detections_result(classification);
    inside_detections_result = find_unused_detections(detections_2d, outside_detections_result);

    return classification;
}

std::size_t TurningScenarioClassifier::find_end_segment_index(const std::vector<cv::Point2d>& convex_hull) {
    std::optional<std::size_t> end_segment_idx{std::nullopt};

    cv::Point2d previous_point{convex_hull[0U].x - 1.0,
                               convex_hull[0U].y};  // Arbitrary to make sure segment starts directly behind
    for (std::size_t idx{0U}; idx < convex_hull.size() - 1U; ++idx) {
        const cv::Point2d current_point = convex_hull[idx];
        const cv::Point2d next_point = convex_hull[idx + 1U];
        const double a = cv::norm(current_point - previous_point);
        const double b = cv::norm(next_point - current_point);
        const double c = cv::norm(next_point - previous_point);
        const double theta = std::acos((-c * c + a * a + b * b) / (2.0 * a * b));  // Law of cosines
        if (theta < params.end_segment_angle_threshold) {
            end_segment_idx = idx;
            break;
        }
        previous_point = current_point;
    }

    if (not end_segment_idx.has_value()) {
        end_segment_idx = find_idx_of_max_distance(convex_hull);
    }

    return end_segment_idx.value();
}

std::size_t TurningScenarioClassifier::find_idx_of_max_distance(const std::vector<cv::Point2d>& convex_hull) {
    std::vector<double> distances{};
    std::transform(std::cbegin(convex_hull), std::cend(convex_hull), std::back_inserter(distances),
                   [](const auto& point) { return cv::norm(point); });
    const auto max_it = std::max_element(std::cbegin(distances), std::cend(distances));
    return static_cast<std::size_t>(std::distance(std::cbegin(distances), max_it));
}

std::vector<cv::Point2d> TurningScenarioClassifier::find_unused_detections(
    const std::vector<cv::Point2d>& all_detections, const std::vector<cv::Point2d>& used_detections) {
    std::vector<cv::Point2d> unused_detections{all_detections};

    for (const auto& used_point : used_detections) {
        const auto it = std::find_if(
            std::begin(unused_detections), std::end(unused_detections),
            [&used_point](const auto& p) { return (used_point.x - p.x) < epsilon && (used_point.y - p.y) < epsilon; });
        unused_detections.erase(it);
    }
    return unused_detections;
}

void TurningScenarioClassifier::associate_unused_detections(std::vector<cv::Point2d>& used_detections,
                                                            const std::vector<cv::Point2d>& unused_detections) {
    for (const cv::Point2d& unused_point : unused_detections) {
        for (std::size_t used_det_idx{0}; used_det_idx < used_detections.size() - 1U; ++used_det_idx) {
            const cv::Point2d p1{used_detections[used_det_idx]};
            const cv::Point2d p2{used_detections[used_det_idx + 1U]};
            const cv::Point2d used_vector{p2 - p1};
            if (cv::norm(used_vector) < epsilon) {  // Shouldn't happen but just in case
                break;
            }
            const cv::Point2d unused_vector{unused_point - p1};

            const double unused_dot_product = unused_vector.dot(used_vector);
            const double used_dot_product = used_vector.dot(used_vector);
            double dot_ratio = unused_dot_product / used_dot_product;
            clamp_between_0_and_1(dot_ratio);
            const cv::Point2d closest_point{used_vector * dot_ratio + p1};

            const double distance = cv::norm(unused_point - closest_point);
            if (distance < params.cluster_threshold) {
                used_detections.push_back(unused_point);
                break;
            }
        }
    }
}
