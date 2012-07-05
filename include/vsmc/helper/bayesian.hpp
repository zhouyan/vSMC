#ifndef VSMC_HELPER_BAYESIAN_HPP
#define VSMC_HELPER_BAYESIAN_HPP

template <unsigned ID>
struct BayesianParamID
{
    enum {value = ID};
}; // class ParamID

class BayesianParam
{
    public :

    BayesianParam () :
        log_prior_(0), log_likelihood_(0), log_target_(0),
        log_prior_old_(0), log_likelihood_old_(0), log_target_old_(0) {}

    double &param (unsigned d)
    {
        return param_[d];
    }

    double param (unsigned d) const
    {
        return param_[d];
    }

    template <typename Param>
    double &param ()
    {
        return param_[Param::value];
    }

    template <typename Param>
    double param () const
    {
        return param_[Param::value];
    }

    double &scale (unsigned d)
    {
        return scale_[d];
    }

    double scale (unsigned d) const
    {
        return scale_[d];
    }

    template <typename scale>
    double &scale ()
    {
        return scale_[scale::value];
    }

    template <typename scale>
    double scale () const
    {
        return scale_[scale::value];
    }

    double &log_prior ()
    {
        return log_prior_;
    }

    double log_prior () const
    {
        return log_prior_;
    }

    double &log_likelihood ()
    {
        return log_likelihood_;
    }

    double log_likelihood () const
    {
        return log_likelihood_;
    }

    double &log_target ()
    {
        return log_target_;
    }

    double log_target () const
    {
        return log_target_;
    }

    double log_target_diff () const
    {
        return log_target_ - log_target_old_;
    }

    double &alpha ()
    {
        return alpha_;
    }

    double alpha () const
    {
        return alpha_;
    }

    double alpha_inc ()
    {
        return alpha_inc_;
    }

    double alpha_inc () const
    {
        return alpha_inc_;
    }

    void param_num (unsigned num)
    {
        param_.resize(num);
        scale_.resize(num);
    }

    unsigned param_num () const
    {
        return param_.size();
    }

    template <typename InputIter>
    void save_old (InputIter first, InputIter last)
    {
        log_prior_old_       = log_prior_;
        log_likelihood_old_  = log_likelihood_;
        log_target_old_      = log_target_;
        for (InputIter i = first; i != last; ++i)
            param_old_[*i] = param_[*i];
    }

    template <typename InputIter>
    void mh_reject (double p, double u, InputIter first, InputIter last)
    {
        if (p < u) {
            log_prior_       = log_prior_old_;
            log_likelihood_  = log_likelihood_old_;
            log_target_      = log_target_old_;
            for (InputIter i = first; i != last; ++i)
                param_[*i] = param_old_[*i];
        }
    }

    private :

    Eigen::VectorXd param_;
    double log_prior_;
    double log_likelihood_;
    double log_target_;

    Eigen::VectorXd param_old_;
    double log_prior_old_;
    double log_likelihood_old_;
    double log_target_old_;

    Eigen::VectorXd scale_;
    double alpha_;
    double alpha_inc_;
}; // class BayesianParam

template <template <typename, typename, typename> class State, typename Timer>
class BayesianState : public State<1, BayesianParam, Timer>
{
    public :

    typedef State<1, BayesianParam, Timer> state_base_type;
    typedef typename state_base_type::size_type size_type;
    typedef Eigen::MatrixXd data_type;
    typedef internal::function<double (const BayesianParam &)>
        log_prior_eval_type;
    typedef internal::function<double (const BayesianParam &,
            const data_type &)> log_likelihood_eval_type;

    explicit BayesianState (size_type N) : state_base_type(N) {}

    void alpha (double a)
    {
        a = a < 1 ? a : 1;
        a = a > 0 ? a : 0;
        double a_inc = a > 0 ? a - this->state(0, 0).alpha() : 0;
        unsigned n = this->size();
        for (unsigned i = 0; i != n; ++i) {
            this->state(i, 0).alpha() = a;
            this->state(i, 0).alpha_inc() = a_inc;
        }
    }

    void param_num (unsigned num)
    {
        const unsigned n = this->size();
        for (unsigned i = 0; i != n; ++i)
            this->state(i, 0).param_num(num);
    }

    double log_prior (log_prior_eval_type &eval)
    {
        log_prior_eval_ = eval;
    }

    double log_likelihood (log_likelihood_eval_type &eval)
    {
        log_likelihood_eval_ = eval;
    }

    double log_prior (BayesianParam &param) const
    {
        return param.log_prior() = log_prior_eval_(param);
    }

    double log_likelihood (BayesianParam &param) const
    {
        return param.log_likelihood() = log_likelihood_eval_(param, data_);
    }

    double log_target (BayesianParam &param) const
    {
        return param.log_target() =
            log_prior(param) + param.alpha() * log_likelihood(param);
    }

    data_type &data ()
    {
        return data_;
    }

    const data_type &data () const
    {
        return data_;
    }

    // TODO We really need a much more flexible read_data
    void read_data (const char *filename, size_type rows, size_type cols)
    {
        std::ifstream input;
        input.open(filename);
        if (!input) {
            input.close();
            input.clear();
            throw std::runtime_error("Failed to open data file");
        }

        data_.resize(rows, cols);
        for (size_type r = 0; r != rows; ++r)
            for (size_type c = 0; c != cols; ++c)
                input >> data_[r, c];
    }

    private :

    data_type data_;
    log_prior_eval_type log_prior_eval_;
    log_likelihood_eval_type log_likelihood_eval_;
}; // class BayesianState

template <typename T, template <typename, typename> Move, typename Scale>
class BayesianMove
{
    public :

    typedef internal::function<unsigned (BayesianParam &)> move_type;
    typedef std::deque<param_move_type> move_queue_type;

    unsigned move_state (unsigned iter, SingleParticle<T> sp)
    {
        unsigned accept;

        for (move_queue_type::iterator m = move_queue_.begin();
                m != move_queue_.end(); ++m)
            accept += (*m)(iter, sp);

        return accept;
    }

    void pre_processor (unsigned iter, Particle<T> &particle)
    {
        scale_(iter, particle.value());
    }

    private :

    Scale scale_;
    move_queue_type move_queue_;
}

#endif // VSMC_HELPER_BAYESIAN_HPP
