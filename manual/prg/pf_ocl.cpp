#include <vsmc/vsmc.hpp>

static constexpr std::size_t N = 10000; // Number of particles
static constexpr std::size_t n = 100;   // Number of data points
static constexpr std::size_t PosX = 0;
static constexpr std::size_t PosY = 1;
static constexpr std::size_t VelX = 2;
static constexpr std::size_t VelY = 3;

static constexpr std::size_t G = 10240;
static constexpr std::size_t L = 256;

typedef struct {
    cl_float pos_x;
    cl_float pos_y;
    cl_float vel_x;
    cl_float vel_y;
} cl_pf_sp;

using PFStateBase = vsmc::StateMatrix<vsmc::RowMajor, 1, cl_pf_sp>;

class PFState : public PFStateBase
{
    public:
    using size_type = cl_int;
    using PFStateBase::StateMatrix;

    void initialize(const vsmc::CLContext &context,
        const vsmc::CLCommandQueue &command_queue,
        const vsmc::CLKernel &kernel)
    {
        command_queue_ = command_queue;
        kernel_ = kernel;

        dev_data_ =
            vsmc::CLMemory(context, CL_MEM_READ_WRITE | CL_MEM_HOST_READ_ONLY,
                sizeof(cl_pf_sp) * size());
        dev_weight_ =
            vsmc::CLMemory(context, CL_MEM_WRITE_ONLY | CL_MEM_HOST_READ_ONLY,
                sizeof(cl_float) * size());
        dev_rng_set_ =
            vsmc::CLMemory(context, CL_MEM_WRITE_ONLY | CL_MEM_HOST_READ_ONLY,
                sizeof(vsmc_threefry4x32) * size());
        dev_index_ =
            vsmc::CLMemory(context, CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY,
                sizeof(cl_int) * size());
        dev_obs_x_ = vsmc::CLMemory(context,
            CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, sizeof(cl_float) * n);
        dev_obs_y_ = vsmc::CLMemory(context,
            CL_MEM_READ_ONLY | CL_MEM_HOST_WRITE_ONLY, sizeof(cl_float) * n);
    }

    void copy(std::size_t N, const cl_int *index)
    {
        command_queue_.enqueue_write_buffer(dev_index_, CL_TRUE, 0,
            sizeof(cl_int) * N, const_cast<cl_int *>(index));

        kernel_.set_arg(0, static_cast<cl_int>(size()));
        kernel_.set_arg(1, dev_data_);
        kernel_.set_arg(2, dev_index_);

        command_queue_.enqueue_nd_range_kernel(kernel_, 1, vsmc::CLNDRange(),
            vsmc::CLNDRange(G), vsmc::CLNDRange(L));
        command_queue_.finish();
    }

    void copy_to_host()
    {
        command_queue_.enqueue_read_buffer(
            dev_data_, CL_TRUE, 0, sizeof(cl_pf_sp) * size(), data());
    }

    void read_data(const char *param)
    {
        if (param == nullptr)
            return;

        vsmc::Vector<cl_float> obs_x(n);
        vsmc::Vector<cl_float> obs_y(n);
        std::ifstream data(param);
        for (std::size_t i = 0; i != n; ++i)
            data >> obs_x[i] >> obs_y[i];
        data.close();

        command_queue_.enqueue_write_buffer(
            dev_obs_x_, CL_TRUE, 0, sizeof(cl_float) * n, obs_x.data());
        command_queue_.enqueue_write_buffer(
            dev_obs_y_, CL_TRUE, 0, sizeof(cl_float) * n, obs_y.data());
    }

    const vsmc::CLMemory &dev_data() const { return dev_data_; }
    const vsmc::CLMemory &dev_weight() const { return dev_weight_; }
    const vsmc::CLMemory &dev_rng_set() const { return dev_rng_set_; }
    const vsmc::CLMemory &dev_obs_x() const { return dev_obs_x_; }
    const vsmc::CLMemory &dev_obs_y() const { return dev_obs_y_; }

    private:
    vsmc::CLCommandQueue command_queue_;
    vsmc::CLKernel kernel_;
    vsmc::CLMemory dev_data_;
    vsmc::CLMemory dev_rng_set_;
    vsmc::CLMemory dev_weight_;
    vsmc::CLMemory dev_index_;
    vsmc::CLMemory dev_obs_x_;
    vsmc::CLMemory dev_obs_y_;
};

class PFInit
{
    public:
    PFInit(const vsmc::CLCommandQueue &command_queue,
        const vsmc::CLKernel &kernel)
        : command_queue_(command_queue), kernel_(kernel)
    {
    }

    std::size_t operator()(vsmc::Particle<PFState> &particle, void *param)
    {
        particle.value().read_data(static_cast<const char *>(param));

        kernel_.set_arg(0, static_cast<cl_int>(particle.size()));
        kernel_.set_arg(1, particle.value().dev_data());
        kernel_.set_arg(2, particle.value().dev_rng_set());
        kernel_.set_arg(3, particle.value().dev_weight());
        kernel_.set_arg(4, particle.value().dev_obs_x());
        kernel_.set_arg(5, particle.value().dev_obs_y());

        command_queue_.enqueue_nd_range_kernel(kernel_, 1, vsmc::CLNDRange(),
            vsmc::CLNDRange(G), vsmc::CLNDRange(L));
        command_queue_.finish();

        weight_.resize(particle.size());
        command_queue_.enqueue_read_buffer(particle.value().dev_weight(),
            CL_TRUE, 0, sizeof(cl_float) * particle.size(), weight_.data());
        particle.weight().set_log(weight_.data());

        return 0;
    }

    private:
    vsmc::CLCommandQueue command_queue_;
    vsmc::CLKernel kernel_;
    vsmc::Vector<cl_float> weight_;
};

class PFMove
{
    public:
    PFMove(const vsmc::CLCommandQueue &command_queue,
        const vsmc::CLKernel &kernel)
        : command_queue_(command_queue), kernel_(kernel)
    {
    }

    std::size_t operator()(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        kernel_.set_arg(0, static_cast<cl_int>(t));
        kernel_.set_arg(1, static_cast<cl_int>(particle.size()));
        kernel_.set_arg(2, particle.value().dev_data());
        kernel_.set_arg(3, particle.value().dev_rng_set());
        kernel_.set_arg(4, particle.value().dev_weight());
        kernel_.set_arg(5, particle.value().dev_obs_x());
        kernel_.set_arg(6, particle.value().dev_obs_y());

        command_queue_.enqueue_nd_range_kernel(kernel_, 1, vsmc::CLNDRange(),
            vsmc::CLNDRange(G), vsmc::CLNDRange(L));
        command_queue_.finish();

        weight_.resize(particle.size());
        command_queue_.enqueue_read_buffer(particle.value().dev_weight(),
            CL_TRUE, 0, sizeof(cl_float) * particle.size(), weight_.data());
        particle.weight().add_log(weight_.data());

        return 0;
    }

    private:
    vsmc::CLCommandQueue command_queue_;
    vsmc::CLKernel kernel_;
    vsmc::Vector<cl_float> weight_;
};

class PFEval : public vsmc::MonitorEvalTBB<PFState, PFEval>
{
    public:
    void eval_pre(std::size_t t, vsmc::Particle<PFState> &particle)
    {
        particle.value().copy_to_host();
    }

    void eval_sp(std::size_t t, std::size_t dim,
        vsmc::SingleParticle<PFState> sp, double *r)
    {
        r[0] = sp.state(0).pos_x;
        r[1] = sp.state(0).pos_y;
    }
};

int main()
{
    auto platform = vsmc::cl_get_platform().front();
    std::string platform_name;
    platform.get_info(CL_PLATFORM_NAME, platform_name);
    std::cout << "Platform: " << platform_name << std::endl;

    auto device = platform.get_device(CL_DEVICE_TYPE_DEFAULT).front();
    std::string device_name;
    device.get_info(CL_DEVICE_NAME, device_name);
    std::cout << "Device:   " << device_name << std::endl;

    vsmc::CLContext context(vsmc::CLContextProperties(platform), 1, &device);
    vsmc::CLCommandQueue command_queue(context, device);

    std::ifstream source_cl("pf_ocl.cl");
    std::string source((std::istreambuf_iterator<char>(source_cl)),
        std::istreambuf_iterator<char>());
    source_cl.close();
    vsmc::CLProgram program(context, 1, &source);
    program.build(1, &device, "-I ../../include");

    vsmc::CLKernel kernel_copy(program, "copy");
    vsmc::CLKernel kernel_init(program, "init");
    vsmc::CLKernel kernel_move(program, "move");

    std::size_t pwgsm_copy = 0;
    std::size_t pwgsm_init = 0;
    std::size_t pwgsm_move = 0;

    kernel_copy.get_work_group_info(
        device, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, pwgsm_copy);
    kernel_init.get_work_group_info(
        device, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, pwgsm_init);
    kernel_move.get_work_group_info(
        device, CL_KERNEL_PREFERRED_WORK_GROUP_SIZE_MULTIPLE, pwgsm_move);

    std::cout << "Kernel copy preferred work group size multiple: "
              << pwgsm_copy << std::endl;
    std::cout << "Kernel init preferred work group size multiple: "
              << pwgsm_init << std::endl;
    std::cout << "Kernel move preferred work group size multiple: "
              << pwgsm_move << std::endl;

    vsmc::Sampler<PFState> sampler(N);
    sampler.resample_method(vsmc::Multinomial, 0.5);
    sampler.particle().value().initialize(context, command_queue, kernel_copy);
    sampler.init(PFInit(command_queue, kernel_init));
    sampler.move(PFMove(command_queue, kernel_move));
    sampler.monitor("pos", 2, PFEval());

    vsmc::StopWatch watch;
    watch.start();
    sampler.initialize(const_cast<char *>("pf.data")).iterate(n - 1);
    watch.stop();
    std::cout << "Time (ms): " << watch.milliseconds() << std::endl;

    std::ofstream output("pf.out");
    output << sampler;
    output.close();

    return 0;
}
