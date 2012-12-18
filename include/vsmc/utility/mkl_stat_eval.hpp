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
        weight_(size_), buffer_(size_), task_(NULL), est_(0), monitor_dim_(0),
        dat_store_(VSL_SS_MATRIX_STORAGE_COLS),
        cov_store_(VSL_SS_MATRIX_STORAGE_FULL)
    {
        int status = vsldSSNewTask(&task_, &dim_, &size_, &dat_store_,
                &buffer_[0], &weight_[0], NULL);
        VSMC_RUNTIME_ASSERT((status == VSL_STATUS_OK),
                "CALLING **vsldSSNewTask** failed");
        edit_task();
    }

    MKLStatEval (const MKLStatEval<T> &other) :
        size_(other.size_), dim_(other.dim_), eval_(other.eval_),
        weight_(size_), buffer_(size_), est_(other.est_),
        monitor_dim_(other.monitor_dim_),
        dat_store_(VSL_SS_MATRIX_STORAGE_COLS),
        cov_store_(VSL_SS_MATRIX_STORAGE_FULL)
    {
        int status = vsldSSNewTask(&task_, &dim_, &size_, &dat_store_,
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
        edit_est_();
    }

    unsigned monitor_dim () const
    {
        return monitor_dim_;
    }

    const std::vector<std::string> &monitor_var_name () const
    {
        return monitor_var_name_;
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
        int status = vsldSSCompute(task_, est_, VSL_SS_METHOD_FAST);
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
    std::vector<std::string> monitor_var_name_;
    MKL_INT dat_store_;
    MKL_INT cov_store_;

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

    void edit_est_ ()
    {
        monitor_dim_ = 0;
        monitor_var_name_.clear();
        std::stringstream ss;

        if (est_ & VSL_SS_MEAN) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "mean." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_2R_MOM) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "r2mom." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_3R_MOM) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "r3mom." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_4R_MOM) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "r4mom." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_2C_MOM) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "var." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_3C_MOM) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "c3mom." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_4C_MOM) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "c4mom." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_KURTOSIS) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "kurtosis." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_SKEWNESS) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "skewness." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_MIN) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "min." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_MAX) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "max." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_VARIATION) {
            monitor_dim_ += dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                ss.str(std::string());
                ss << "variation." << i + 1;
                monitor_var_name_.push_back(ss.str());
            }
        }

        if (est_ & VSL_SS_COV) {
            monitor_dim_ += dim_ * dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                for (MKL_INT j = 0; j != dim_; ++j) {
                    ss.str(std::string());
                    ss << "cov." << i + 1 << "." << j + 1;
                    monitor_var_name_.push_back(ss.str());
                }
            }
        }

        if (est_ & VSL_SS_COR) {
            monitor_dim_ += dim_ * dim_;
            for (MKL_INT i = 0; i != dim_; ++i) {
                for (MKL_INT j = 0; j != dim_; ++j) {
                    ss.str(std::string());
                    ss << "cor." << i + 1 << "." << j + 1;
                    monitor_var_name_.push_back(ss.str());
                }
            }
        }
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
            status = vsliSSEditTask(task_, VSL_SS_ED_COV_STORAGE, &cov_store_);
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);

            status = vsldSSEditTask(task_, VSL_SS_ED_COV, res + offset);
            offset += dim_ * dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }

        if (est_ & VSL_SS_COR) {
            status = vsliSSEditTask(task_, VSL_SS_ED_COR_STORAGE, &cov_store_);
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);

            status = vsldSSEditTask(task_, VSL_SS_ED_COR, res + offset);
            offset += dim_ * dim_;
            VSMC_RUNTIME_ASSERT_MKL_STAT_EVAL_EDIT_TASK(status);
        }
    }
}; // class MKLStatEval

}

#endif // VSMC_UTILITY_MKL_STAT_EVAL_HPP
