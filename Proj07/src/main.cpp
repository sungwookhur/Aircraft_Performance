#include <iostream>
#include <fstream>
#include <cmath>

// -------------------------------
// 상태 및 파라미터 정의
// -------------------------------

struct State {
    double theta; // pitch angle [rad]
    double q;     // pitch rate [rad/s]
};

struct Params {
    // 관성 및 모멘트 계수
    double Iy;   // pitch axis moment of inertia
    double a1;   // aerodynamic damping w.r.t q
    double a2;   // linear stiffness w.r.t theta
    double a3;   // nonlinear stiffness (theta^3 term)
    double b1;   // control effectiveness (moment per delta_e)

    // 비선형 제어기 게인
    double k1;       // proportional gain
    double k2;       // derivative gain
    double k3;       // nonlinear (cubic) gain
    double theta_cmd; // desired pitch angle [rad]
};

// -------------------------------
// 연속 비선형 모델: xdot = f(x, delta_e)
// -------------------------------
State aircraft_dynamics(const State& x, double delta_e, const Params& p)
{
    State dx;

    // theta_dot = q
    dx.theta = x.q;

    // 비선형 공기력 모멘트 (예시)
    double M_aero  = -p.a1 * x.q - p.a2 * x.theta - p.a3 * std::pow(x.theta, 3);

    // 제어 모멘트
    double M_ctrl  = p.b1 * delta_e;

    // q_dot = (M_aero + M_ctrl) / Iy
    dx.q = (M_aero + M_ctrl) / p.Iy;

    return dx;
}

// -------------------------------
// 비선형 상태피드백 제어기
//   e = theta - theta_cmd
//   delta_e = -k1 * e - k2 * q - k3 * e^3
// -------------------------------
double nonlinear_controller(const State& x, const Params& p)
{
    double e = x.theta - p.theta_cmd;
    double delta_e = -p.k1 * e - p.k2 * x.q - p.k3 * e * e * e;
    return delta_e;
}

// -------------------------------
// RK4 적분 한 스텝
//   x_{k+1} = x_k + h/6 (k1 + 2k2 + 2k3 + k4)
//   여기서는 입력 delta_e는 샘플 동안 상수(ZOH)라고 가정
// -------------------------------
State rk4_step(const State& x, double delta_e, double t, double h, const Params& p)
{
    (void)t; // t를 모델에서 쓰지 않으므로 unused warning 방지용

    State k1 = aircraft_dynamics(x, delta_e, p);

    State x2;
    x2.theta = x.theta + 0.5 * h * k1.theta;
    x2.q     = x.q     + 0.5 * h * k1.q;
    State k2 = aircraft_dynamics(x2, delta_e, p);

    State x3;
    x3.theta = x.theta + 0.5 * h * k2.theta;
    x3.q     = x.q     + 0.5 * h * k2.q;
    State k3 = aircraft_dynamics(x3, delta_e, p);

    State x4;
    x4.theta = x.theta + h * k3.theta;
    x4.q     = x.q     + h * k3.q;
    State k4 = aircraft_dynamics(x4, delta_e, p);

    State x_next;
    x_next.theta = x.theta + (h / 6.0) * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
    x_next.q     = x.q     + (h / 6.0) * (k1.q     + 2.0 * k2.q     + 2.0 * k3.q     + k4.q);

    return x_next;
}

// -------------------------------
// 메인: 시뮬레이션 루프
// -------------------------------
int main()
{
    // 파라미터 설정 (예시값)
    Params p;

    // 관성 및 모멘트 파라미터
    p.Iy = 1.0;    // [kg m^2] 예시
    p.a1 = 0.8;    // q에 대한 감쇠
    p.a2 = 2.0;    // theta에 대한 선형 복원
    p.a3 = 5.0;    // theta^3 비선형 복원
    p.b1 = 1.0;    // delta_e -> 모멘트 비례 상수

    // 제어기 게인 (조금 튜닝이 필요한 예시값)
    p.k1 = 4.0;   // P 게인
    p.k2 = 3.0;   // D 게인
    p.k3 = 1.0;   // 비선형(e^3) 게인

    // 목표 피치각 (10 deg 정도)
    p.theta_cmd = 10.0 * M_PI / 180.0;

    // 시뮬레이션 설정
    double dt     = 0.01; // 10 ms -> 100 Hz 샘플링
    double T_end  = 10.0; // 10초 시뮬레이션
    int    N_step = static_cast<int>(T_end / dt);

    // 초기 상태: 약간 다른 각도에서 시작 (예: -5 deg)
    State x;
    x.theta = -5.0 * M_PI / 180.0; // [rad]
    x.q     = 0.0;                 // [rad/s]

    double t = 0.0;

    // 결과를 파일로 저장 (CSV)
    std::ofstream ofs("pitch_sim.csv");
    if (!ofs.is_open()) {
        std::cerr << "Cannot open output file.\n";
        return 1;
    }

    ofs << "t,theta_rad,theta_deg,q,delta_e\n";

    for (int k = 0; k <= N_step; ++k) {
        // 1) 현재 상태에서 비선형 제어기로 delta_e 계산
        double delta_e = nonlinear_controller(x, p);

        // 결과 기록
        double theta_deg = x.theta * 180.0 / M_PI;
        ofs << t << "," << x.theta << "," << theta_deg << "," << x.q << "," << delta_e << "\n";

        // 2) RK4로 연속 비선형 동역학 한 스텝 적분
        State x_next = rk4_step(x, delta_e, t, dt, p);

        // 시간 및 상태 업데이트
        x = x_next;
        t += dt;
    }

    ofs.close();

    std::cout << "Simulation finished. Results saved to pitch_sim.csv\n";
    std::cout << "You can plot theta_deg vs t in Python/Matlab/Excel etc.\n";

    return 0;
}
