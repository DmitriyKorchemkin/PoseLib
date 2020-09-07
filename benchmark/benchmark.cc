#include "benchmark.h"
#include "problem_generator.h"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

namespace pose_lib {

template <typename Solver>
BenchmarkResult benchmark(int n_problems, const ProblemOptions &options, double tol = 1e-6) {

    std::vector<AbsolutePoseProblemInstance> problem_instances;
    generate_abspose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;
    std::vector<double> errors;
    errors.reserve(n_problems);

    // Run benchmark where we check solution quality
    for (const AbsolutePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;

        int sols = Solver::solve(instance, &solutions);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        //std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        bool found_valid = false;
        for (const CameraPose &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, tol)) {
                result.valid_solutions_++;
                found_valid = true;
            }
            //std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose));
        }
        if (found_valid)
          result.at_least_one_valid_++;

        errors.push_back(pose_error);
        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const AbsolutePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    auto med_runtime = runtimes.begin() + runtimes.size() / 2;
    std::nth_element(runtimes.begin(), med_runtime, runtimes.end());
    result.runtime_ns_ = *med_runtime;

    auto prev_quantile = errors.begin();
    auto tgt_quantile = errors.begin() + errors.size() * 0.5;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_med = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.9;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q90 = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.95;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q95 = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.99;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q99 = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.999;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q999 = *tgt_quantile;


    std::cout << "\r                                                                                \r";
    return result;
}

template <typename Solver>
BenchmarkResult benchmark_relative(int n_problems, const ProblemOptions &options, double tol = 1e-6) {

    std::vector<RelativePoseProblemInstance> problem_instances;
    generate_relpose_problems(n_problems, &problem_instances, options);

    BenchmarkResult result;
    result.instances_ = n_problems;
    result.name_ = Solver::name();
    if (options.additional_name_ != "") {
        result.name_ += options.additional_name_;
    }
    result.options_ = options;
    std::cout << "Running benchmark: " << result.name_ << std::flush;

    std::vector<double> errors;
    errors.reserve(n_problems);
    // Run benchmark where we check solution quality
    for (const RelativePoseProblemInstance &instance : problem_instances) {
        CameraPoseVector solutions;

        int sols = Solver::solve(instance, &solutions);

        double pose_error = std::numeric_limits<double>::max();

        result.solutions_ += sols;
        //std::cout << "Gt: " << instance.pose_gt.R << "\n"<< instance.pose_gt.t << "\n";
        bool found_valid = false;
        for (const CameraPose &pose : solutions) {
            if (Solver::validator::is_valid(instance, pose, tol)) {
                result.valid_solutions_++;
                found_valid = true;
            }
            //std::cout << "Pose: " << pose.R << "\n" << pose.t << "\n";
            pose_error = std::min(pose_error, Solver::validator::compute_pose_error(instance, pose));
        }
        if (found_valid)
          ++result.at_least_one_valid_;

        errors.push_back(pose_error);
        if (pose_error < tol)
            result.found_gt_pose_++;
    }

    std::vector<long> runtimes;
    CameraPoseVector solutions;
    for (int iter = 0; iter < 10; ++iter) {
        int total_sols = 0;
        auto start_time = std::chrono::high_resolution_clock::now();
        for (const RelativePoseProblemInstance &instance : problem_instances) {
            solutions.clear();

            int sols = Solver::solve(instance, &solutions);

            total_sols += sols;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        runtimes.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count());
    }

    auto med_runtime = runtimes.begin() + runtimes.size() / 2;
    std::nth_element(runtimes.begin(), med_runtime, runtimes.end());
    result.runtime_ns_ = *med_runtime;

    auto prev_quantile = errors.begin();
    auto tgt_quantile = errors.begin() + errors.size() * 0.5;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_med = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.9;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q90 = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.95;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q95 = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.99;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q99 = *tgt_quantile;

    prev_quantile = tgt_quantile;
    tgt_quantile = errors.begin() + errors.size() * 0.999;
    std::nth_element(prev_quantile, tgt_quantile, errors.end());
    result.err_q999 = *tgt_quantile;

    std::cout << "\r                                                                                \r";
    return result;
}

} // namespace pose_lib

void print_runtime(double runtime_ns) {
    if (runtime_ns < 1e3) {
        std::cout << runtime_ns << " ns";
    } else if (runtime_ns < 1e6) {
        std::cout << runtime_ns / 1e3 << " us";
    } else if (runtime_ns < 1e9) {
        std::cout << runtime_ns / 1e6 << " ms";
    } else {
        std::cout << runtime_ns / 1e9 << " s";
    }
}

void display_result(const std::vector<pose_lib::BenchmarkResult> &results) {
    // Print PoseLib version and buidling type
    std::cout << "\n" << poselib_info() << "\n\n";

    int w = 16;
    // display header
    std::cout << std::setw(2 * w) << "Solver";
    std::cout << std::setw(w) << "Solutions";
    std::cout << std::setw(w) << "Valid";
    std::cout << std::setw(w) << "1+ valid";
    std::cout << std::setw(w) << "GT found";
    std::cout << std::setw(w) << "0.5 quantile";
    std::cout << std::setw(w) << "0.9 quantile";
    std::cout << std::setw(w) << "0.95 quantile";
    std::cout << std::setw(w) << "0.99 quantile";
    std::cout << std::setw(w) << "0.999 quantile";
    std::cout << std::setw(w) << "Runtime"
              << "\n";
    for (int i = 0; i < w * 12; ++i)
        std::cout << "-";
    std::cout << "\n";

    int prec = 6;

    for (const pose_lib::BenchmarkResult &result : results) {
        double num_tests = static_cast<double>(result.instances_);
        double solutions = result.solutions_ / num_tests;
        double valid_sols = result.valid_solutions_ / static_cast<double>(result.solutions_) * 100.0;
        double at_least_one = result.at_least_one_valid_ / num_tests * 100.;
        double gt_found = result.found_gt_pose_ / num_tests * 100.0;
        double runtime_ns = result.runtime_ns_ / num_tests;
        double q05 = result.err_med;
        double q09 = result.err_q90;
        double q095 = result.err_q95;
        double q099 = result.err_q99;
        double q0999 = result.err_q999;

        std::cout << std::setprecision(prec) << std::setw(2 * w) << result.name_;
        std::cout << std::setprecision(prec) << std::setw(w) << solutions;
        std::cout << std::setprecision(prec) << std::setw(w) << valid_sols;
        std::cout << std::setprecision(prec) << std::setw(w) << at_least_one;
        std::cout << std::setprecision(prec) << std::setw(w) << gt_found;
        std::cout << std::setprecision(prec) << std::setw(w) << q05;
        std::cout << std::setprecision(prec) << std::setw(w) << q09;
        std::cout << std::setprecision(prec) << std::setw(w) << q095;
        std::cout << std::setprecision(prec) << std::setw(w) << q099;
        std::cout << std::setprecision(prec) << std::setw(w) << q0999;
        std::cout << std::setprecision(prec) << std::setw(w - 3);
        print_runtime(runtime_ns);
        std::cout << "\n";
    }
}

int main() {

    std::vector<pose_lib::BenchmarkResult> results;

    pose_lib::ProblemOptions options;
    // options.camera_fov_ = 45; // Narrow
    // options.camera_fov_ = 75; // Medium
    options.camera_fov_ = 120; // Wide

    double tol = 1e-6;

    // P3P
    pose_lib::ProblemOptions p3p_opt = options;
    p3p_opt.n_point_point_ = 3;
    p3p_opt.n_point_line_ = 0;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP3P>(1e5, p3p_opt, tol));

    // gP3P
    pose_lib::ProblemOptions gp3p_opt = options;
    gp3p_opt.n_point_point_ = 3;
    gp3p_opt.n_point_line_ = 0;
    gp3p_opt.generalized_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverGP3P>(1e4, gp3p_opt, tol));

    // gP4Ps
    pose_lib::ProblemOptions gp4p_opt = options;
    gp4p_opt.n_point_point_ = 4;
    gp4p_opt.n_point_line_ = 0;
    gp4p_opt.generalized_ = true;
    gp4p_opt.unknown_scale_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverGP4PS>(1e4, gp4p_opt, tol));

    // gP4Ps Quasi-degenerate case (same 3D point observed twice)
    gp4p_opt.generalized_duplicate_obs_ = true;
    gp4p_opt.additional_name_ = "(Deg.)";
    results.push_back(pose_lib::benchmark<pose_lib::SolverGP4PS>(1e4, gp4p_opt, tol));

    // P4Pf
    pose_lib::ProblemOptions p4pf_opt = options;
    p4pf_opt.n_point_point_ = 4;
    p4pf_opt.n_point_line_ = 0;
    p4pf_opt.unknown_focal_ = true;
#define P4PF_TEST(companion, orth, filter) \
    results.push_back(pose_lib::benchmark<pose_lib::SolverP4PF<companion,orth,pose_lib::p4pf_filter::filter>>(1e4, p4pf_opt, tol));
#define P4PF_TEST_SOLVERS(orth, filter) \
    P4PF_TEST(false, orth, filter);\
    P4PF_TEST(true , orth, filter);
#define P4PF_TEST_ORTH(filter) \
    P4PF_TEST_SOLVERS(false, filter); \
    P4PF_TEST_SOLVERS(true, filter);
#define P4PF_TEST_FILTERS \
    P4PF_TEST_ORTH(norm); \
    P4PF_TEST_ORTH(singular_values); \
    P4PF_TEST_ORTH(reprojection);

    P4PF_TEST_FILTERS;

    // P2P2PL
    pose_lib::ProblemOptions p2p2pl_opt = options;
    p2p2pl_opt.n_point_point_ = 2;
    p2p2pl_opt.n_point_line_ = 2;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP2P2PL>(1e3, p2p2pl_opt, tol));

    // P6LP
    pose_lib::ProblemOptions p6lp_opt = options;
    p6lp_opt.n_line_point_ = 6;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP6LP>(1e4, p6lp_opt, tol));

    // P5LP Radial
    pose_lib::ProblemOptions p5lp_radial_opt = options;
    p5lp_radial_opt.n_line_point_ = 5;
    p5lp_radial_opt.radial_lines_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP5LP_Radial>(1e5, p5lp_radial_opt, tol));

    // P2P1LL
    pose_lib::ProblemOptions p2p1ll_opt = options;
    p2p1ll_opt.n_point_point_ = 2;
    p2p1ll_opt.n_line_line_ = 1;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP2P1LL>(1e4, p2p1ll_opt, tol));

    // P1P2LL
    pose_lib::ProblemOptions p1p2ll_opt = options;
    p1p2ll_opt.n_point_point_ = 1;
    p1p2ll_opt.n_line_line_ = 2;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP1P2LL>(1e4, p1p2ll_opt, tol));

    // P3LL
    pose_lib::ProblemOptions p3ll_opt = options;
    p3ll_opt.n_line_line_ = 3;
    results.push_back(pose_lib::benchmark<pose_lib::SolverP3LL>(1e4, p3ll_opt, tol));
    // uP2P
    pose_lib::ProblemOptions up2p_opt = options;
    up2p_opt.n_point_point_ = 2;
    up2p_opt.n_point_line_ = 0;
    up2p_opt.upright_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverUP2P>(1e6, up2p_opt, tol));

    // uGP2P
    pose_lib::ProblemOptions ugp2p_opt = options;
    ugp2p_opt.n_point_point_ = 2;
    ugp2p_opt.n_point_line_ = 0;
    ugp2p_opt.upright_ = true;
    ugp2p_opt.generalized_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverUGP2P>(1e6, ugp2p_opt, tol));

    // uGP3Ps
    pose_lib::ProblemOptions ugp3ps_opt = options;
    ugp3ps_opt.n_point_point_ = 3;
    ugp3ps_opt.n_point_line_ = 0;
    ugp3ps_opt.upright_ = true;
    ugp3ps_opt.generalized_ = true;
    ugp3ps_opt.unknown_scale_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverUGP3PS>(1e5, ugp3ps_opt, tol));

    // uP1P2PL
    pose_lib::ProblemOptions up1p2pl_opt = options;
    up1p2pl_opt.n_point_point_ = 1;
    up1p2pl_opt.n_point_line_ = 2;
    up1p2pl_opt.upright_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverUP1P2PL>(1e4, up1p2pl_opt, tol));

    // uP4PL
    pose_lib::ProblemOptions up4pl_opt = options;
    up4pl_opt.n_point_point_ = 0;
    up4pl_opt.n_point_line_ = 4;
    up4pl_opt.upright_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverUP4PL>(1e4, up4pl_opt, tol));

    // ugP4PL
    pose_lib::ProblemOptions ugp4pl_opt = options;
    ugp4pl_opt.n_point_point_ = 0;
    ugp4pl_opt.n_point_line_ = 4;
    ugp4pl_opt.upright_ = true;
    ugp4pl_opt.generalized_ = true;
    results.push_back(pose_lib::benchmark<pose_lib::SolverUGP4PL>(1e4, ugp4pl_opt, tol));

    // Relative Pose Upright
    pose_lib::ProblemOptions relupright3pt_opt = options;
    relupright3pt_opt.n_point_point_ = 3;
    relupright3pt_opt.upright_ = true;
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRelUpright3pt>(1e4, relupright3pt_opt, tol));

    // Generalized Relative Pose Upright
    pose_lib::ProblemOptions genrelupright4pt_opt = options;
    genrelupright4pt_opt.n_point_point_ = 4;
    genrelupright4pt_opt.upright_ = true;
    genrelupright4pt_opt.generalized_ = true;
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverGenRelUpright4pt>(1e4, genrelupright4pt_opt, tol));

    // Relative Pose 8pt
    pose_lib::ProblemOptions rel8pt_opt = options;
    rel8pt_opt.n_point_point_ = 8;
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRel8pt>(1e4, rel8pt_opt, tol));

    rel8pt_opt.additional_name_ = "(100 pts)";
    rel8pt_opt.n_point_point_ = 100;
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRel8pt>(1e4, rel8pt_opt, tol));

    // Relative Pose 5pt
    pose_lib::ProblemOptions rel5pt_opt = options;
    rel5pt_opt.n_point_point_ = 5;
    // companion matrix eigenvalues
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRel5pt<true>>(1e4, rel5pt_opt, tol));
    // sturm bracketing
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRel5pt<false>>(1e4, rel5pt_opt, tol));


    // Relative Pose Upright Planar 2pt
    pose_lib::ProblemOptions reluprightplanar2pt_opt = options;
    reluprightplanar2pt_opt.n_point_point_ = 2;
    reluprightplanar2pt_opt.upright_ = true;
    reluprightplanar2pt_opt.planar_ = true;
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRelUprightPlanar2pt>(1e4, reluprightplanar2pt_opt, tol));

    // Relative Pose Upright Planar 3pt
    pose_lib::ProblemOptions reluprightplanar3pt_opt = options;
    reluprightplanar3pt_opt.n_point_point_ = 3;
    reluprightplanar3pt_opt.upright_ = true;
    reluprightplanar3pt_opt.planar_ = true;
    results.push_back(pose_lib::benchmark_relative<pose_lib::SolverRelUprightPlanar3pt>(1e4, reluprightplanar3pt_opt, tol));

    display_result(results);

    return 0;
}
