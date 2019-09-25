/* Author: Saifullah */

#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>
#include <vector>

template <typename Scalar, int Dim, int Degree = 3>
class SplineInterpolation;
template <typename Scalar>
class SlerpInterpolation;

template <typename Scalar, int Dim, int Degree = 3>
struct CartesianInterpolation {
  CartesianInterpolation(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Dim>& positions,
    const std::vector<Eigen::Quaternion<Scalar>>& orientations,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& knots
  ) {
    position = new SplineInterpolation<Scalar, Dim, Degree>(positions, knots);
    orientation = new SlerpInterpolation<Scalar>(orientations, knots);
  }

  ~CartesianInterpolation() {
    delete position;
    delete orientation;
  }

  SplineInterpolation<Scalar, Dim, Degree>* position;
  SlerpInterpolation<Scalar>* orientation;
};

template <typename Scalar, int Dim, int Degree>
class SplineInterpolation {
public:
  SplineInterpolation(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Dim>& points,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& knots)
  {
    knot_min_ = knots.minCoeff();
    knot_diff_ = knots.maxCoeff() - knot_min_;
    spline_ = 
      Eigen::SplineFitting<Eigen::Spline<Scalar, Dim>>::Interpolate(
        points, Degree, normalize(knots)
      );
  }
    
  Eigen::Matrix<Scalar, Dim, 1> operator()(const Scalar& t) const {
    // time need to be normalized in extraction as well.
    return spline_(normalize(t));
  }

private:
  Scalar normalize(const Scalar& knot) const {
    return (knot - knot_min_) / knot_diff_;
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> normalize(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& knots) const {
    return knots.unaryExpr([this](Scalar knot) { return normalize(knot); });
  }

  Scalar knot_min_;
  Scalar knot_diff_;
  Eigen::Spline<Scalar, Dim> spline_;
};

template <typename Scalar>
class SlerpInterpolation {
public:
  SlerpInterpolation(
    const std::vector<Eigen::Quaternion<Scalar>>& orientations,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& knots) :
    knots_(knots), orientations_(orientations)
  {
    knots_diff_.resize(knots.rows()-1);
    for (int i = 1; i < knots.size(); ++i) {
      knots_diff_[i-1] = knots_[i] - knots_[i-1];
    }
  }
    
  Eigen::Quaternion<Scalar> operator()(const Scalar& t) const {
    int knotIndex = 0;
    for (int i = 1; i < knots_.size(); ++i) { // index 0 is always time = 0.0
      if (t < knots_[i]) {
        knotIndex = i;
        break;
      }
    }
    auto norm_t = normalize(t, knotIndex);
    auto& q = orientations_[knotIndex-1];
    return q.slerp(norm_t, orientations_[knotIndex]);
  }

private:
  Scalar normalize(const Scalar& t, const int& knotIndex) const {
    return (t - knots_[knotIndex-1]) / knots_diff_[knotIndex-1];
  }

  std::vector<Eigen::Quaternion<Scalar>> orientations_;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> knots_;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> knots_diff_;
};