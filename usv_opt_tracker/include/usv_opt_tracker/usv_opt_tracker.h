// Main Contributor:Garron Liu
// Date: 13th July 2024
// Great thanks to the authors of control toolbox https://github.com/ethz-adrl/control-toolbox for providing such a
// powerful common nonlinear optimal control tools

#ifndef USV_OPT_TRACKER_H
#define USV_OPT_TRACKER_H

#include <ct/optcon/optcon.h>
#include <ct/core/core.h>
#include <ros/ros.h>
#include <iostream>
#include "exampleDir.h"

#include "UsvDynamicSystem.h"
#define AutoDiff

using namespace ct::core;
using namespace ct::optcon;
using namespace std;

namespace usv_system
{
const size_t state_dim = usv::USVCorrectionModel::STATE_DIM;  // x y cos(psi) sin(psi) psi  u v r
const size_t control_dim = usv::USVCorrectionModel::CONTROL_DIM;
const size_t velocity_dim = usv::USVCorrectionModel::VELOCITY_STATE_DIM;  // u v r

#ifdef AutoDiffCodegen
using Scalar = ct::core::ADCGScalar;
#endif
#ifdef AutoDiff
using Scalar = ct::core::ADScalar;
#endif

class UsvOptTracker
{
public:
  typedef std::shared_ptr<usv::GaussianProcessModel> GPModelPtr_t;
  typedef std::shared_ptr<usv::tpl::GaussianProcessModel<velocity_dim, control_dim, Scalar>> ADGPModelPtr_t;
  typedef std::shared_ptr<usv::USVDynamicModel> NominalSystemPtr_t;
  typedef std::shared_ptr<usv::tpl::USVDynamicModel<Scalar>> NominalADSystemPtr_t;
  typedef std::shared_ptr<usv::USVCorrectionModel> SystemPtr_t;
  typedef std::shared_ptr<usv::tpl::USVCorrectionModel<Scalar>> ADSystemPtr_t;
  typedef std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> LinearSystemPtr_t;
#ifdef AutoDiffCodegen
  typedef std::shared_ptr<ct::core::ADCodegenLinearizer<state_dim, control_dim>> ADLinearSystemPtr_t;
#endif
#ifdef AutoDiff
  typedef std::shared_ptr<ct::core::AutoDiffLinearizer<state_dim, control_dim>> ADLinearSystemPtr_t;
#endif
  typedef std::shared_ptr<ct::optcon::NLOptConSolver<state_dim, control_dim>> NLOPPtr_t;
  typedef std::shared_ptr<ct::optcon::MPC<NLOptConSolver<state_dim, control_dim>>> MPCPtr_t;
  typedef std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>> CostFunctionPtr_t;
  typedef std::shared_ptr<ct::optcon::TermQuadTracking<state_dim, control_dim>> TermQuadTrackingPtr_t;
  typedef std::shared_ptr<ct::optcon::TermQuadratic<state_dim, control_dim>> TermQuadPtr_t;
  typedef std::shared_ptr<ConstraintContainerAnalytical<state_dim, control_dim>> ConstraintPtr_t;
  typedef ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t NLOPPolicy_t;
  typedef ct::core::StateFeedbackController<state_dim, control_dim> Controller_t;

  UsvOptTracker(bool GPREnable_ = false) : GPREnable(GPREnable_)
  {
    // initialize dynamic system
    trackingCost.reset(new TermQuadTracking<state_dim, control_dim>());
    trackingCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost.info", "trackingCost", true);
    finalCost.reset(new TermQuadratic<state_dim, control_dim>());
    finalCost->loadConfigFile(ct::optcon::exampleDir + "/nlocCost.info", "finalCost", true);
    nloc_settings.load(ct::optcon::exampleDir + "/nlocSolver.info", true, "ilqr");
    ct::optcon::loadMpcSettings(ct::optcon::exampleDir + "/mpcSolver.info", mpc_settings);
    timeHorizon = mpc_settings.minimumTimeHorizonMpc_;

    usv_settings = usv::tpl::loadSettings<double>(ct::optcon::exampleDir + "/model.info", "USVParameters", true);

    usv_nominal_dynamic_.reset(new usv::USVDynamicModel(usv_settings));
    if (GPREnable)
    {
      gp_dynamics.reset(new usv::GaussianProcessModel(ct::optcon::exampleDir));
      usv_dynamic_.reset(new usv::USVCorrectionModel(usv_nominal_dynamic_, gp_dynamics));
    }

    // usv::tpl::USVParameters<Scalar> usv_settings_AD;
    // usv_settings_AD = usv::tpl::loadSettings<Scalar>(ct::optcon::exampleDir + "/model.info", "USVParameters", false);

    usv_nominal_dynamic_AD_.reset(new usv::tpl::USVDynamicModel<Scalar>(usv_settings));
    if (GPREnable)
    {
      gp_dynamics_AD.reset(
          new usv::tpl::GaussianProcessModel<velocity_dim, control_dim, Scalar>(ct::optcon::exampleDir));
      usv_dynamic_AD.reset(new usv::tpl::USVCorrectionModel<Scalar>(usv_nominal_dynamic_AD_, gp_dynamics_AD));
    }
#ifdef AutoDiffCodegen
    if (GPREnable)
    {
      ADLinearizer.reset(new ct::core::ADCodegenLinearizer<state_dim, control_dim>(usv_dynamic_AD));
    }
    else
    {
      ADLinearizer.reset(new ct::core::ADCodegenLinearizer<state_dim, control_dim>(usv_nominal_dynamic_AD_));
    }
    auto start = std::chrono::high_resolution_clock::now();
    ADLinearizer->compileJIT();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "JIT compilation took " << duration.count() << " microseconds." << std::endl;
#endif

#ifdef AutoDiff
    if (GPREnable)
    {
      ADLinearizer.reset(new ct::core::AutoDiffLinearizer<state_dim, control_dim>(usv_dynamic_AD));
    }
    else
    {
      ADLinearizer.reset(new ct::core::AutoDiffLinearizer<state_dim, control_dim>(usv_nominal_dynamic_AD_));
    }
#endif

    // constraints of inputs
    Eigen::VectorXd u_lb(control_dim);
    Eigen::VectorXd u_ub(control_dim);

    u_ub << usv_settings.maxInputRate_, usv_settings.maxInputRate_;
    u_lb << -usv_settings.maxInputRate_, -usv_settings.maxInputRate_;

    controlInputBound.reset(new ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub));
    controlInputBound->setName("ControlInputBound");
    inputBoxConstraints.reset(new ConstraintContainerAnalytical<state_dim, control_dim>());
    inputBoxConstraints->addIntermediateConstraint(controlInputBound, true);
    inputBoxConstraints->initialize();

    // constraints of states
    Eigen::VectorXi sp_state(state_dim);
    sp_state << 0, 0, 0, 0, 0, 1, 0, 1, 1, 1;  // limit the surge velocity and yaw rate
    Eigen::VectorXd x_lb(4);
    Eigen::VectorXd x_ub(4);
    x_lb << usv_settings.minSurgeSpeed_, -usv_settings.maxAngularSpeed_, -usv_settings.maxInput_,
        -usv_settings.maxInput_;
    x_ub << usv_settings.maxSurgeSpeed_, usv_settings.maxAngularSpeed_, usv_settings.maxInput_, usv_settings.maxInput_;
    stateBound.reset(new StateConstraint<state_dim, control_dim>(x_lb, x_ub, sp_state));
    stateBound->setName("StateBound");
    stateBoxConstraints.reset(new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());
    stateBoxConstraints->addIntermediateConstraint(stateBound, true);
    stateBoxConstraints->initialize();

    mpc_initialized = false;
  }
  ~UsvOptTracker()
  {
  }

  /* call update_mpc_tracker while trajectory reference updation ignited */
  void update_mpc_tracker(const ct::core::StateVector<state_dim>& x_init,
                          const ct::core::StateTrajectory<state_dim>& x_ref_traj,
                          const ct::core::ControlTrajectory<control_dim>& u_ref_traj)
  {
    // update current reference trajectory in horizon
    this->trackingCost->setStateAndControlReference(x_ref_traj, u_ref_traj);

    ct::core::StateVector<state_dim> x_ref_final;
    x_ref_final = x_ref_traj.back();
    ct::core::ControlVector<control_dim> u_ref_final;
    u_ref_final.setZero();
    this->finalCost->setStateAndControlReference(x_ref_final, u_ref_final);

    // combine intermediate cost and final cost
    this->costCombinedFunction.reset(new CostFunctionAnalytical<state_dim, control_dim>());
    this->costCombinedFunction->addIntermediateTerm(this->trackingCost);
    this->costCombinedFunction->addFinalTerm(this->finalCost);

    if (!mpc_initialized)
    {
      if (GPREnable)
      {
        ct::optcon::ContinuousOptConProblem<state_dim, control_dim> optConProblem(
            this->timeHorizon, x_init, this->usv_dynamic_, this->costCombinedFunction, this->ADLinearizer);
        optConProblem.setInputBoxConstraints(this->inputBoxConstraints);
        optConProblem.setStateBoxConstraints(this->stateBoxConstraints);

        size_t K = nloc_settings.computeK(timeHorizon);
        ct::core::FeedbackArray<state_dim, control_dim> u0_fb(K,
                                                              ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
        ct::core::ControlVectorArray<control_dim> u0_ff(K, ct::core::ControlVector<control_dim>::Zero());
        ct::core::StateVectorArray<state_dim> x_ref_init(K + 1, x_init);

        NLOPPolicy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

        this->nloc.reset(new ct::optcon::NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
        // // set the initial guess
        this->nloc->setInitialGuess(initController);
        //  solve the optimal control problem
        this->nloc->solve();
        Controller_t solution = this->nloc->getSolution();

        // init mpc solver
        NLOptConSettings nloc_settings_mpc = nloc_settings;
        nloc_settings_mpc.max_iterations = 10;

        this->mpc.reset(new ct::optcon::MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, nloc_settings_mpc,
                                                                                    mpc_settings));
        this->mpc->setInitialGuess(solution);
        mpc_start_time = ros::Time::now().toSec();
        mpc_initialized = true;
      }
      else
      {
        ct::optcon::ContinuousOptConProblem<state_dim, control_dim> optConProblem(
            this->timeHorizon, x_init, this->usv_nominal_dynamic_, this->costCombinedFunction, this->ADLinearizer);
        optConProblem.setInputBoxConstraints(this->inputBoxConstraints);
        optConProblem.setStateBoxConstraints(this->stateBoxConstraints);

        size_t K = nloc_settings.computeK(timeHorizon);
        ct::core::FeedbackArray<state_dim, control_dim> u0_fb(K,
                                                              ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
        ct::core::ControlVectorArray<control_dim> u0_ff(K, ct::core::ControlVector<control_dim>::Zero());
        ct::core::StateVectorArray<state_dim> x_ref_init(K + 1, x_init);

        NLOPPolicy_t initController(x_ref_init, u0_ff, u0_fb, nloc_settings.dt);

        this->nloc.reset(new ct::optcon::NLOptConSolver<state_dim, control_dim>(optConProblem, nloc_settings));
        // // set the initial guess
        this->nloc->setInitialGuess(initController);
        //  solve the optimal control problem
        this->nloc->solve();
        Controller_t solution = this->nloc->getSolution();

        // init mpc solver
        NLOptConSettings nloc_settings_mpc = nloc_settings;
        nloc_settings_mpc.max_iterations = 10;

        this->mpc.reset(new ct::optcon::MPC<NLOptConSolver<state_dim, control_dim>>(optConProblem, nloc_settings_mpc,
                                                                                    mpc_settings));
        this->mpc->setInitialGuess(solution);
        mpc_start_time = ros::Time::now().toSec();
        mpc_initialized = true;
      }
    }
    else
    {
      auto solver = this->mpc->getSolver();
      solver.changeCostFunction(this->costCombinedFunction);
      mpc_start_time = ros::Time::now().toSec();
    }
  }

  ct::core::ControlVector<control_dim> getMPCControl(const ct::core::StateVector<state_dim>& x_now,
                                                     ct::core::StateVectorArray<state_dim>& x_pred_traj)
  {
    if (!mpc_initialized)
      return ct::core::ControlVector<control_dim>::Zero();
    auto time_start = std::chrono::high_resolution_clock::now();
    ct::core::Time current_time = ros::Time::now().toSec();
    ct::core::Time t = current_time - mpc_start_time;

    this->mpc->prepareIteration(t);

    current_time = ros::Time::now().toSec();
    t = current_time - mpc_start_time;

    this->mpc->finishIteration(x_now, t, newSolution, ts_newSoltuion);

    current_time = ros::Time::now().toSec();
    t = current_time - mpc_start_time;
    ct::core::ControlVector<control_dim> u;

    newSolution.computeControl(x_now, t - ts_newSoltuion, u);
    x_pred_traj = newSolution.x_ref();

    auto time_end = std::chrono::high_resolution_clock::now();
    auto duration_occ = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start);
    if (nloc_settings.printSummary)
    {
      cout << "NMPC solver spend: " << static_cast<double>(duration_occ.count()) / 1000.0 << " ms" << endl;
      printf("current optimal control:(%.5f,%.5f)", u(0), u(1));
    }

    return u;
  }

  void getGPTrainingPoints(const ct::core::StateVector<state_dim>& x_last,
                           const ct::core::StateVector<state_dim>& x_observe,
                           const ct::core::ControlVector<control_dim>& u_opt, double dt, Eigen::VectorXd& Targets)
  {
    /* integrate from last state to get nominal prediction Bv_k+1 at time k+1 with optimal input u_opt_k */
    ct::core::StateVector<state_dim> nextNominalState(x_last);

    // set up a controller for the manually integrated system
    std::shared_ptr<ct::core::ConstantController<state_dim, control_dim>> constantController(
        new ct::core::ConstantController<state_dim, control_dim>());
    usv_nominal_dynamic_->setController(constantController);

    // parameters for discretization
    double startTime = 0.0;
    size_t K_sim = 20;

    // integrators for the nominal system
    ct::core::Integrator<state_dim> integratorRK4(usv_nominal_dynamic_, ct::core::RK4);
    // integrate the nominal system
    constantController->setControl(u_opt);
    integratorRK4.integrate_n_steps(nextNominalState, startTime, K_sim, dt / (double)K_sim);

    /* calculate the acceleration error */
    ct::core::StateVector<state_dim> accError;
    accError = (x_observe - nextNominalState) / dt;
    Targets(0) = accError(5);
    Targets(1) = accError(6);
    Targets(2) = accError(7);
    Targets(3) = x_observe(5);
    Targets(4) = x_observe(6);
    Targets(5) = x_observe(7);
    Targets(6) = x_last(8);
    Targets(7) = x_last(9);
    // std::cout<<"Acceleration Error as GP Targets:"<<Targets<<endl;
  }

  void updateGP(const std::vector<Eigen::VectorXd>& gpTargets)
  {
    gp_dynamics->updateTrainingData(gpTargets);
    gp_dynamics_AD->updateTrainingData(gpTargets);
    std::cout << "GP updated!" << std::endl;
  }

  double getMaxVelocity()
  {
    return this->usv_settings.maxSurgeSpeed_;
  }
  double getMpcHorizon()
  {
    return this->timeHorizon;
  }
  double getTimeInterval()
  {
    return this->nloc_settings.dt;
  }

private:
  std::string nloc_settingfile_directory;
  std::string nloc_costfile_directory;

  NominalSystemPtr_t usv_nominal_dynamic_;
  NominalADSystemPtr_t usv_nominal_dynamic_AD_;
  SystemPtr_t usv_dynamic_;
  ADSystemPtr_t usv_dynamic_AD;
  GPModelPtr_t gp_dynamics;
  ADGPModelPtr_t gp_dynamics_AD;

  LinearSystemPtr_t Linearizer;
  ADLinearSystemPtr_t ADLinearizer;

  TermQuadTrackingPtr_t trackingCost;
  TermQuadPtr_t finalCost;
  CostFunctionPtr_t costCombinedFunction;
  ct::optcon::NLOptConSettings nloc_settings;
  ct::optcon::mpc_settings mpc_settings;
  usv::tpl::USVParameters<double> usv_settings;

  NLOPPtr_t nloc;
  MPCPtr_t mpc;

  // constraint terms
  std::shared_ptr<ControlInputConstraint<state_dim, control_dim>> controlInputBound;
  std::shared_ptr<StateConstraint<state_dim, control_dim>> stateBound;

  // input and state box constraint constraint container
  ConstraintPtr_t inputBoxConstraints;
  ConstraintPtr_t stateBoxConstraints;
  double max_vel, max_rot_vel, max_con_input;
  double min_vel;
  ct::core::Time timeHorizon;
  ct::core::Time mpc_start_time;
  ct::core::Time ts_newSoltuion;
  bool mpc_initialized;

  Controller_t newSolution;

  bool GPREnable;
};
}  // namespace usv_system
#endif
