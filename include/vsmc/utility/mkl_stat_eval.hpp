#ifndef VSMC_UTILITY_MKL_STAT_EVAL_HPP
#define VSMC_UTILITY_MKL_STAT_EVAL_HPP

#include <vsmc/internal/common.hpp>
#include <mkl_vsl.h>

namespace vsmc {

template <typename T>
class MKLStatEval
{
    public :

    typedef T value_type;
    typedef cxx11::function<void (
            unsigned, unsigned, const Particle<T> &, double *)> eval_type;

    explicit MKLStatEval (unsigned dim, const eval_type &eval) :
        size_(2), dim_(static_cast<MKL_INT>(dim)), eval_(eval),
        weight_(size_), buffer_(size_), task_(NULL), est_(0),
        monitor_dim_(0), storage_(VSL_SS_MATRIX_STORAGE_COLS)
    {
        int status = vsldSSNewTask(&task_, &dim_, &size_, &storage_,
                &buffer_[0], &weight_[0], NULL);
        VSMC_RUNTIME_ASSERT((status == VSL_STATUS_OK),
                "CALLING **vsldSSNewTask** failed");
        edit_task();
    }

    MKLStatEval (const MKLStatEval<T> &other) :
        size_(other.size_), dim_(other.dim_), eval_(other.eval_),
        weight_(size_), buffer_(size_), est_(other.est_),
        monitor_dim_(other.monitor_dim_), storage_(VSL_SS_MATRIX_STORAGE_COLS)
    {
        int status = vsldSSNewTask(&task_, &dim_, &size_, &storage_,
                &buffer_[0], &weight_[0], NULL);
        VSMC_RUNTIME_ASSERT((status == VSL_STATUS_OK),
                "CALLING **vsldSSNewTask** failed");
        edit_task();
    }

    MKLStatEval<T> &operator= (const MKLStatEval<T> &other)
    {
        if (this != &other) {
            size_ = other.size_;
            dim_ = other.dim_;
            eval_ = other.eval_;
            weight_.resize(size_);
            buffer_.resize(size_);
            est_ = other.est_;
            monitor_dim_ = other.monitor_dim_;
            edit_task();
        }

        return *this;
    }

    ~MKLStatEval ()
    {
        int status = vslSSDeleteTask(&task_);
        VSMC_RUNTIME_ASSERT((status == VSL_STATUS_OK),
                "CALLING **vsldSSDeleteTask** failed");
    }

    void set_estimator (MKL_INT64 est)
    {
        est_ = est;
        edit_monitor_dim();
    }

    unsigned monitor_dim () const
    {
        return monitor_dim_;
    }

    void operator() (unsigned iter, unsigned dim,
            const Particle<T> &particle, double *res)
    {
        VSMC_RUNTIME_ASSERT((dim >= monitor_dim_),
                "DIMENSION OF OUTPUT PARAMETER IN **vsmc::MKLStatEval::eval**"
                "TOO SMALL");

        if (size_ != static_cast<MKL_INT>(particle.size())) {
            size_ = static_cast<MKL_INT>(particle.size());
            weight_.resize(size_);
            buffer_.resize(size_ * dim_);
            edit_task();
        }

        eval_(iter, dim_, particle, &buffer_[0]);
        particle.read_weight(&weight_[0]);

        if (!monitor_dim_)
            return;

        edit_output(res);
        int status = vsldSSCompute(task_, est_, VSL_SS_METHOD_1PASS);
        VSMC_RUNTIME_ASSERT((status == VSL_STATUS_OK),
                "CALLING **vsldSSComputeTask** failed");
    }

    private :

    MKL_INT size_;
    MKL_INT dim_;
    eval_type eval_;
    std::vector<double> weight_;
    std::vector<double> buffer_;
    VSLSSTaskPtr task_;
    MKL_INT64 est_;
    unsigned monitor_dim_;
    MKL_INT storage_;

    void edit_task ()
    {
        int status = VSL_STATUS_OK;

        status = vsliSSEditTask(task_, VSL_SS_ED_OBSERV_N, &size_);
        VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);

        status = vsldSSEditTask(task_, VSL_SS_ED_OBSERV, &buffer_[0]);
        VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);

        status = vsldSSEditTask(task_, VSL_SS_ED_WEIGHTS, &weight_[0]);
        VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
    }

    void edit_monitor_dim ()
    {
        MKL_INT64 d = 0;
        if (est_ & VSL_SS_MEAN)      d += dim_;
        if (est_ & VSL_SS_2R_MOM)    d += dim_;
        if (est_ & VSL_SS_3R_MOM)    d += dim_;
        if (est_ & VSL_SS_4R_MOM)    d += dim_;
        if (est_ & VSL_SS_2C_MOM)    d += dim_;
        if (est_ & VSL_SS_3C_MOM)    d += dim_;
        if (est_ & VSL_SS_4C_MOM)    d += dim_;
        if (est_ & VSL_SS_KURTOSIS)  d += dim_;
        if (est_ & VSL_SS_SKEWNESS)  d += dim_;
        if (est_ & VSL_SS_MIN)       d += dim_;
        if (est_ & VSL_SS_MAX)       d += dim_;
        if (est_ & VSL_SS_VARIATION) d += dim_;
        if (est_ & VSL_SS_COV)       d += dim_ * dim_;
        if (est_ & VSL_SS_COR)       d += dim_ * dim_;
        monitor_dim_ = static_cast<unsigned>(d);
    }

    void edit_output (double *res)
    {
        int status = VSL_STATUS_OK;
        std::size_t offset = 0;

        if (est_ & VSL_SS_MEAN) {
            status = vsldSSEditTask(task_, VSL_SS_ED_MEAN, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_2R_MOM) {
            status = vsldSSEditTask(task_, VSL_SS_ED_2R_MOM, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_3R_MOM) {
            status = vsldSSEditTask(task_, VSL_SS_ED_3R_MOM, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_4R_MOM) {
            status = vsldSSEditTask(task_, VSL_SS_ED_4R_MOM, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_2C_MOM) {
            status = vsldSSEditTask(task_, VSL_SS_ED_2C_MOM, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_3C_MOM) {
            status = vsldSSEditTask(task_, VSL_SS_ED_3C_MOM, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_4C_MOM) {
            status = vsldSSEditTask(task_, VSL_SS_ED_4C_MOM, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_KURTOSIS) {
            status = vsldSSEditTask(task_, VSL_SS_ED_KURTOSIS, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_SKEWNESS) {
            status = vsldSSEditTask(task_, VSL_SS_ED_SKEWNESS, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_MIN) {
            status = vsldSSEditTask(task_, VSL_SS_ED_MIN, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_MAX) {
            status = vsldSSEditTask(task_, VSL_SS_ED_MAX, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_VARIATION) {
            status = vsldSSEditTask(task_, VSL_SS_ED_VARIATION, res + offset);
            offset += dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_COV) {
            status = vsldSSEditTask(task_, VSL_SS_ED_COV, res + offset);
            offset += dim_ * dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_COR) {
            status = vsldSSEditTask(task_, VSL_SS_ED_COR, res + offset);
            offset += dim_ * dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }
    }
}; // class MKLStatEval

}

#endif // VSMC_UTILITY_MKL_STAT_EVAL_HPP
