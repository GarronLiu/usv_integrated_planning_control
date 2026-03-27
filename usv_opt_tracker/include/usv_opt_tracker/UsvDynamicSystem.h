#pragma once

#include <ct/core/core.h>

#include <cmath>
#include <iostream>
#include <memory>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace std;

namespace usv
{
namespace tpl
{
template <typename SCALAR>
struct USVParameters
{
  // For safety, these parameters cannot be modified
  SCALAR Xu_;
  SCALAR Xu_absu_;
  SCALAR Xvr_;
  SCALAR Xrr_;
  SCALAR Yv_;
  SCALAR Yr_;
  SCALAR Yv_absv_;
  SCALAR Yr_absr_;
  SCALAR Yuv_;
  SCALAR Yur_;
  SCALAR Yabsv_r_;
  SCALAR Yv_absr_;
  SCALAR Mv_;
  SCALAR Mr_;
  SCALAR Mv_absv_;
  SCALAR Mr_absr_;
  SCALAR Muv_;
  SCALAR Mur_;
  SCALAR Mabsv_r_;
  SCALAR Mv_absr_;
  SCALAR Thrust_l_;
  SCALAR Thrust_r_;
  SCALAR Side_l_;
  SCALAR Side_r_;
  SCALAR Torque_l_;
  SCALAR Torque_r_;

  SCALAR maxInput_;
  SCALAR maxInputRate_;
  SCALAR maxSurgeSpeed_;
  SCALAR minSurgeSpeed_;
  SCALAR maxAngularSpeed_;
};

template <typename T>
static inline void printValue(std::ostream& stream, const T& value, const std::string& name, bool updated = true,
                              long printWidth = 80)
{
  const std::string nameString = " #### '" + name + "'";
  stream << nameString;

  printWidth = std::max<long>(printWidth, nameString.size() + 15);
  stream.width(printWidth - nameString.size());
  const char fill = stream.fill('.');

  if (updated)
  {
    stream << value << '\n';
  }
  else
  {
    stream << value << " (default)\n";
  }

  stream.fill(fill);
}

template <typename T>
inline void loadPtreeValue(const boost::property_tree::ptree& pt, T& value, const std::string& name, bool verbose,
                           long printWidth = 80)
{
  bool updated = true;

  try
  {
    value = pt.get<T>(name);
  }
  catch (const boost::property_tree::ptree_bad_path&)
  {
    updated = false;
  }

  if (verbose)
  {
    const std::string nameString = name.substr(name.find_last_of('.') + 1);
    printValue(std::cout, value, nameString, updated, printWidth);
  }
}

template <typename SCALAR>
inline USVParameters<SCALAR> loadSettings(const std::string& filename, const std::string& fieldName = "USVParameters",
                                          bool verbose = true)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  USVParameters<SCALAR> settings;

  if (verbose)
  {
    std::cout << "\n #### USV Parameters:";
    std::cout << "\n #### =============================================================================\n";
  }

  loadPtreeValue(pt, settings.Xu_, fieldName + ".Xu", verbose);
  loadPtreeValue(pt, settings.Xu_absu_, fieldName + ".Xu_absu", verbose);
  loadPtreeValue(pt, settings.Xvr_, fieldName + ".Xvr", verbose);
  loadPtreeValue(pt, settings.Xrr_, fieldName + ".Xrr", verbose);

  loadPtreeValue(pt, settings.Yv_, fieldName + ".Yv", verbose);
  loadPtreeValue(pt, settings.Yr_, fieldName + ".Yr", verbose);
  loadPtreeValue(pt, settings.Yv_absv_, fieldName + ".Yv_absv", verbose);
  loadPtreeValue(pt, settings.Yr_absr_, fieldName + ".Yr_absr", verbose);
  loadPtreeValue(pt, settings.Yuv_, fieldName + ".Yuv", verbose);
  loadPtreeValue(pt, settings.Yur_, fieldName + ".Yur", verbose);
  loadPtreeValue(pt, settings.Yabsv_r_, fieldName + ".Yabsv_r", verbose);
  loadPtreeValue(pt, settings.Yv_absr_, fieldName + ".Yv_absr", verbose);

  loadPtreeValue(pt, settings.Mv_, fieldName + ".Mv", verbose);
  loadPtreeValue(pt, settings.Mr_, fieldName + ".Mr", verbose);
  loadPtreeValue(pt, settings.Mv_absv_, fieldName + ".Mv_absv", verbose);
  loadPtreeValue(pt, settings.Mr_absr_, fieldName + ".Mr_absr", verbose);
  loadPtreeValue(pt, settings.Muv_, fieldName + ".Muv", verbose);
  loadPtreeValue(pt, settings.Mur_, fieldName + ".Mur", verbose);
  loadPtreeValue(pt, settings.Mabsv_r_, fieldName + ".Mabsv_r", verbose);
  loadPtreeValue(pt, settings.Mv_absr_, fieldName + ".Mv_absr", verbose);

  loadPtreeValue(pt, settings.Thrust_l_, fieldName + ".Thrust_l", verbose);
  loadPtreeValue(pt, settings.Thrust_r_, fieldName + ".Thrust_r", verbose);
  loadPtreeValue(pt, settings.Side_l_, fieldName + ".Side_l", verbose);
  loadPtreeValue(pt, settings.Side_r_, fieldName + ".Side_r", verbose);
  loadPtreeValue(pt, settings.Torque_l_, fieldName + ".Torque_l", verbose);
  loadPtreeValue(pt, settings.Torque_r_, fieldName + ".Torque_r", verbose);

  loadPtreeValue(pt, settings.maxInput_, fieldName + ".maxInput", verbose);
  loadPtreeValue(pt, settings.maxInputRate_, fieldName + ".maxInputRate", verbose);
  loadPtreeValue(pt, settings.maxSurgeSpeed_, fieldName + ".maxSurgeSpeed", verbose);
  loadPtreeValue(pt, settings.minSurgeSpeed_, fieldName + ".minSurgeSpeed", verbose);
  loadPtreeValue(pt, settings.maxAngularSpeed_, fieldName + ".maxAngularSpeed", verbose);

  if (verbose)
  {
    std::cout << " #### =============================================================================" << std::endl;
  }

  return settings;
}

template <typename SCALAR>
class USVDynamicModel : public ct::core::ControlledSystem<10, 2, SCALAR>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const size_t STATE_DIM = 10;   // x y cos(psi) sin(psi) psi  u v r
  static const size_t CONTROL_DIM = 2;  // dual thrusts

  USVDynamicModel() = delete;

  USVDynamicModel(const USVParameters<double>& usvParameters) : params_(std::move(usvParameters))
  {
    std::cout << "USV model parameters loaded!" << std::endl;
  }

  USVDynamicModel(const USVDynamicModel<SCALAR>& other) : params_(std::move(other.params_))
  {
  }  //自定义模型时，拷贝构造函数也要有相应的实现，否则求解会有问题

  void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state, const SCALAR& t,
                                 const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
                                 ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override
  {
    // x y cos(psi) sin(psi) psi  u v r
    // SCALAR x = state(0);
    // SCALAR y = state(1);
    SCALAR cpsi = state(2);
    SCALAR spsi = state(3);
    SCALAR psi = state(4);
    SCALAR u = state(5);
    SCALAR v = state(6);
    SCALAR r = state(7);
    SCALAR input_l = state(8);
    SCALAR input_r = state(9);

    SCALAR uu = u * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(u);
    SCALAR vv = v * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v);
    SCALAR rr = r * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(r);
    SCALAR absv_r = r * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(v);
    SCALAR v_absr = v * ct::core::tpl::TraitSelector<SCALAR>::Trait::fabs(r);

    derivative(0) = u * cpsi - v * spsi;
    derivative(1) = u * spsi + v * cpsi;
    derivative(2) = -ct::core::tpl::TraitSelector<SCALAR>::Trait::sin(psi) * r;
    derivative(3) = ct::core::tpl::TraitSelector<SCALAR>::Trait::cos(psi) * r;
    derivative(4) = r;
    derivative(5) = params_.Xu_ * u + params_.Xu_absu_ * uu + params_.Xvr_ * v * r + params_.Xrr_ * r * r +
                    params_.Thrust_l_ * input_l + params_.Thrust_r_ * input_r;
    derivative(6) = params_.Yv_ * v + params_.Yr_ * r + params_.Yv_absv_ * vv + params_.Yr_absr_ * rr +
                    params_.Yuv_ * u * v + params_.Yur_ * u * r + params_.Yabsv_r_ * absv_r +
                    params_.Yv_absr_ * v_absr + params_.Side_l_ * input_l + params_.Side_r_ * input_r;
    derivative(7) = params_.Mv_ * v + params_.Mr_ * r + params_.Mv_absv_ * vv + params_.Mr_absr_ * rr +
                    params_.Muv_ * u * v + params_.Mur_ * u * r + params_.Mabsv_r_ * absv_r +
                    params_.Mv_absr_ * v_absr + params_.Torque_l_ * input_l + params_.Torque_r_ * input_r;
    derivative(8) = control(0);
    derivative(9) = control(1);
  }

  USVDynamicModel* clone() const override
  {
    return new USVDynamicModel(*this);
  }

  ~USVDynamicModel(){};

private:
  // hydrodynamic coefficients
  USVParameters<double> params_;
};

/// @brief use gaussian process regression to model the system dynamics
/// @tparam SCALAR
/// @tparam STATE_DIMS
/// @tparam CONTROL_DIMS
template <size_t STATE_DIMS, size_t CONTROL_DIMS, typename SCALAR>
class GaussianProcessModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = STATE_DIMS;
  static const size_t CONTROL_DIM = CONTROL_DIMS;
  static const size_t INPUT_DIM = STATE_DIM + CONTROL_DIM;
  std::string state_names[STATE_DIM] = { "u", "v", "r" };
  GaussianProcessModel(const std::string& file_prefix, bool verbose = true)
  {
    // initialize std vectors
    inducePointsNum_.resize(STATE_DIM);
    hyperParametersNum_.resize(STATE_DIM);

    hyperParameters.resize(STATE_DIM);

    inducePoints.resize(STATE_DIM);
    induceTargets.resize(STATE_DIM);
    induceCovarianceMatrix.resize(STATE_DIM);

    alpha.resize(STATE_DIM);
    LMatrix.resize(STATE_DIM);

    std::string filename;
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      filename = file_prefix + "/output_" + state_names[dim] + "_sparse.txt";
      std::cout << "loading induce points from " << filename << std::endl;
      std::ifstream infile(filename);
      if (!infile.is_open())
      {
        std::cerr << "Error: cannot open file " << filename << std::endl;
        return;
      }
      int stage = 0;
      std::string s;
      while (infile.good())
      {
        getline(infile, s);
        // ignore empty lines and comments
        if (s.length() != 0 && s.at(0) != '#')
        {
          std::stringstream ss(s);
          if (stage > 2)
          {
            std::string value;
            std::vector<double> row;
            while (std::getline(ss, value, ' '))
            {
              row.push_back(std::stod(value));
            }
            if (row.size() == INPUT_DIM + 1)
            {
              induceTargets[dim].push_back(SCALAR(row[0]));
              inducePoints[dim].push_back((Eigen::Matrix<SCALAR, INPUT_DIM, 1>() << SCALAR(row[1]), SCALAR(row[2]),
                                           SCALAR(row[3]), SCALAR(row[4]), SCALAR(row[5]))
                                              .finished());
            }
          }
          else if (stage == 0)
          {
            // ss >> INPUT_DIM;
          }
          else if (stage == 1)
          {
            // CovFactory factory;
            // cf = factory.create(input_dim, s);
            // cf->loghyper_changed = 0;
          }
          else if (stage == 2)
          {
            Eigen::Matrix<double, Eigen::Dynamic, 1> loghypers(INPUT_DIM + 2);
            for (size_t i = 0; i < INPUT_DIM + 2; ++i)
            {
              ss >> loghypers[i];
            }
            // covSEard loghyperparams: log([length scale, signal variance, noise variance])
            hyperParametersNum_[dim] = INPUT_DIM + 2;
            hyperParameters[dim].resize(hyperParametersNum_[dim]);
            for (size_t i = 0; i < INPUT_DIM; ++i)
            {
              hyperParameters[dim](i) = ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(SCALAR(loghypers[i]));
            }
            hyperParameters[dim](INPUT_DIM) =
                ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(SCALAR(2 * loghypers[INPUT_DIM]));
            hyperParameters[dim](INPUT_DIM + 1) =
                ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(SCALAR(2 * loghypers[INPUT_DIM + 1]));
          }
          stage++;
        }
      }
      infile.close();
      if (stage < 3)
      {
        std::cerr << "fatal error while reading " << filename << std::endl;
        exit(EXIT_FAILURE);
      }
      inducePointsNum_[dim] = inducePoints[dim].size();
      std::cout << inducePointsNum_[dim] << " inducedPoints loaded! " << std::endl;
    }
    updateAlpha_LMatrix();
  }

  void updateHyperParameters();

  void updateTrainingData(const std::vector<Eigen::VectorXd>& gpTargets)
  {
    for (size_t i = 0; i < gpTargets.size(); i++)
    {
      for (size_t dim = 0; dim < STATE_DIM; dim++)
      {
        induceTargets[dim][i] = SCALAR(gpTargets[i](dim));
        inducePoints[dim][i] =
            (Eigen::Matrix<SCALAR, INPUT_DIM, 1>() << SCALAR(gpTargets[i](3)), SCALAR(gpTargets[i](4)),
             SCALAR(gpTargets[i](5)), SCALAR(gpTargets[i](6)), SCALAR(gpTargets[i](7)))
                .finished();
      }
    }
    updateAlpha_LMatrix();
  };

  ~GaussianProcessModel(){};

  void predict(const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented, Eigen::Matrix<SCALAR, STATE_DIM, 1>& output)
  {
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      SCALAR mean = alpha[dim](0) * computeKernel(state_augmented, inducePoints[dim][0], dim);
      for (size_t i = 1; i < inducePointsNum_[dim]; i++)
      {
        mean += alpha[dim](i) * computeKernel(state_augmented, inducePoints[dim][i], dim);
      }
      output(dim) = mean;
    }
  };

  SCALAR covariance(const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented_1,
                    Eigen::Matrix<SCALAR, STATE_DIM, 1>& output)
  {
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      Eigen::Matrix<SCALAR, 1, Eigen::Dynamic> v;
      v.resize(1, inducePointsNum_);
      v.setZero();
      for (size_t i = 0; i < inducePointsNum_[dim]; i++)
      {
        v += computeKernel(state_augmented_1, inducePoints[dim][i], dim) * LMatrix[dim].row(i);
      }
      output(dim) = computeKernel(state_augmented_1, state_augmented_1, dim) - v * v.transpose();
    }
  };

  GaussianProcessModel* clone() const
  {
    return new GaussianProcessModel(*this);
  }

private:
  std::vector<size_t> inducePointsNum_;
  std::vector<size_t> hyperParametersNum_;  // 0: noise variance, 1: signal variance, 2~: length scale

  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> hyperParameters;

  std::vector<std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>>> inducePoints;  // z_ind = (x_ind; u_ind)
  std::vector<std::vector<SCALAR>> induceTargets;                                   // y_ind
  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> induceCovarianceMatrix;

  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> alpha;
  std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>> LMatrix;

  void updateAlpha_LMatrix()
  {
    for (size_t dim = 0; dim < STATE_DIM; dim++)
    {
      Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic> L_Kuu;
      L_Kuu.resize(inducePointsNum_[dim], inducePointsNum_[dim]);
      for (size_t i = 0; i < inducePointsNum_[dim]; ++i)
      {
        for (size_t j = 0; j <= i; ++j)
        {
          L_Kuu(i, j) = computeKernel(inducePoints[dim][i], inducePoints[dim][j], dim);
        }
      }
      L_Kuu.diagonal().array() += hyperParameters[dim](INPUT_DIM + 1);  // noise variance
      L_Kuu = L_Kuu.template selfadjointView<Eigen::Lower>().llt().matrixL();

      Eigen::Map<const Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>> y(&induceTargets[dim][0], induceTargets[dim].size());
      alpha[dim] = L_Kuu.template triangularView<Eigen::Lower>().solve(y);
      L_Kuu.template triangularView<Eigen::Lower>().adjoint().solveInPlace(alpha[dim]);

      // LMatrix[dim] = (L_Kuu * L_Kuu.transpose() - induceCovarianceMatrix).template llt().matrixL();
      // L_Kuu.template triangularView<Eigen::Lower>().solveInPlace(LMatrix[dim]);
      // L_Kuu.template triangularView<Eigen::Lower>().adjoint().solveInPlace(LMatrix[dim]);
    }
  };

  // ard squared exponential kernel
  SCALAR computeKernel(const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented_1,
                       const Eigen::Matrix<SCALAR, INPUT_DIM, 1>& state_augmented_2, size_t dim)
  {
    SCALAR z =
        (state_augmented_1 - state_augmented_2).cwiseQuotient(hyperParameters[dim].head(INPUT_DIM)).squaredNorm();
    return hyperParameters[dim](INPUT_DIM) * ct::core::tpl::TraitSelector<SCALAR>::Trait::exp(-0.5 * z);
  };
};

template <typename SCALAR>
class USVCorrectionModel : public ct::core::ControlledSystem<10, 2, SCALAR>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = 10;  // x y cos(psi) sin(psi) psi  u v r
  static const size_t VELOCITY_STATE_DIM = 3;
  static const size_t CONTROL_DIM = 2;  // dual thrusts

  using GaussianProcessModel_t = GaussianProcessModel<VELOCITY_STATE_DIM, CONTROL_DIM, SCALAR>;
  using USVDynamicModel_t = USVDynamicModel<SCALAR>;

  USVCorrectionModel() = delete;

  USVCorrectionModel(std::shared_ptr<USVDynamicModel_t> usvNominalModel,
                     std::shared_ptr<GaussianProcessModel_t> gpModel)
    : usvNominalModel_(usvNominalModel), gpModel_(gpModel)
  {
    std::cout << "USV Corrected Model Generated!" << std::endl;
  }

  USVCorrectionModel(const USVCorrectionModel<SCALAR>& other)
  {
    if (other.usvNominalModel_)
    {
      usvNominalModel_ = std::shared_ptr<USVDynamicModel_t>(other.usvNominalModel_->clone());
    }
    if (other.gpModel_)
    {
      gpModel_ = std::shared_ptr<GaussianProcessModel_t>(other.gpModel_->clone());
    }
  }  //自定义模型时，拷贝构造函数也要有相应的实现，否则求解会有问题

  void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& state, const SCALAR& t,
                                 const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
                                 ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override
  {
    ct::core::StateVector<VELOCITY_STATE_DIM, SCALAR> gp_correction;
    gpModel_->predict(state.tail(5), gp_correction);

    usvNominalModel_->computeControlledDynamics(state, t, control, derivative);

    derivative(5) += gp_correction(0);
    derivative(6) += gp_correction(1);
    derivative(7) += gp_correction(2);
  }

  USVCorrectionModel* clone() const override
  {
    return new USVCorrectionModel(*this);
  }

  ~USVCorrectionModel(){};

private:
  std::shared_ptr<GaussianProcessModel_t> gpModel_;
  std::shared_ptr<USVDynamicModel_t> usvNominalModel_;
};

}  // namespace tpl
typedef tpl::USVDynamicModel<double> USVDynamicModel;
typedef tpl::USVCorrectionModel<double> USVCorrectionModel;
typedef tpl::GaussianProcessModel<3, 2, double> GaussianProcessModel;

}  // namespace usv
