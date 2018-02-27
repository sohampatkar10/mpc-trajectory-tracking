using namespace std;

class QuadDynamics {
public:
  QuadDynamics(double num_propogation_steps, double h, double num_states, double num_inputs, double num_total_steps) : 
    num_propogation_steps_(num_propogation_steps), h_(h), num_states_(num_states), num_inputs_(num_inputs),
    num_total_steps_(num_total_steps) {}

  void runDynamics(vector<double>& x, const vector<double>& u) {
    for(int i=0; i < num_propogation_steps_-1; i++) {
      vector<double> ut(num_inputs_), ut_1(num_inputs_);
      for(int k=0; k < num_inputs_; k++) {
        ut[k] = u[num_inputs_*i + k];
        ut_1[k] = u[num_inputs_*(i+1) + k];
      }
      runDynamicsSingleStep(x, ut, ut_1);
    }
  }

private:
  double num_propogation_steps_;
  double h_;
  double num_states_;
  double num_inputs_;
  double num_total_steps_;

  void runDynamicsSingleStep(vector<double>& x,const vector<double>& ut,const vector<double>& ut_1) {
    double dt = num_total_steps_*h_;
    int num_int_steps = (int)(1.0/dt);
    for(int k=0; k < num_int_steps; k++) {
      double sx = ut[0] + k*dt*(ut_1[0] - ut[0]);
      double sy = ut[1] + k*dt*(ut_1[1] - ut[1]);
      double sz = ut[2] + k*dt*(ut_1[2] - ut[2]);

      double ga2 = ut[3] + k*dt*(ut_1[3] - ut[3]);

      double qd1 = ut[4] + k*dt*(ut_1[4] - ut[4]);
      double qd2 = ut[5] + k*dt*(ut_1[5] - ut[5]);

      x[9] = x[9] + sx*h_; x[10] = x[10] + sy*h_; x[11] = x[11] + sz*h_;
      x[6] = x[6] + x[9]*h_; x[7] = x[7] + x[10]*h_; x[8] = x[8] + x[11]*h_;
      x[3] = x[3] + x[6]*h_; x[4] = x[4] + x[7]*h_; x[5] = x[5] + x[8]*h_;
      x[0] = x[0] + x[3]*h_; x[1] = x[1] + x[4]*h_; x[2] = x[2] + x[5]*h_;

      x[13] = x[13] + ga2*h_; x[12] = x[12] + x[13]*h_;
      x[14] = x[14] + qd1*h_; x[15] = x[15] + qd2*h_;
    }
  }
};