#include <vsmc/vsmc.hpp>

static constexpr std::size_t N = 1000; // Number of particles
static constexpr std::size_t n = 100;  // Number of particles

using PFStateBase = vsmc::StateMatrix<vsmc::RowMajor, 5, cl_float>;

class PFState : public PFStateBase
{
    public:
    using PFStateBase::StateMatrix;

    void initialize(const vsmc::CLContext &context)
    {
        dev_rng_set_ = vsmc::CLMemory(context, CL_MEM_READ_WRITE,
            sizeof(vsmc_threefry4x32) * size(), nullptr);
        dev_state_ = vsmc::CLMemory(context, CL_MEM_READ_WRITE,
            sizeof(cl_float) * size() * dim(), nullptr);
        dev_obs_x_ = vsmc::CLMemory(
            context, CL_MEM_READ_ONLY, sizeof(cl_float) * n, nullptr);
        dev_obs_y_ = vsmc::CLMemory(
            context, CL_MEM_READ_ONLY, sizeof(cl_float) * n, nullptr);
    }

    void update_host(const vsmc::CLCommandQueue &command_queue)
    {
        command_queue.enqueue_read_buffer(
            dev_state_, CL_TRUE, 0, sizeof(cl_float) * size() * dim(), data());
    }

    void update_device(const vsmc::CLCommandQueue &command_queue)
    {
        command_queue.enqueue_write_buffer(
            dev_state_, CL_TRUE, 0, sizeof(cl_float) * size() * dim(), data());
    }

    void read_data(
        const vsmc::CLCommandQueue &command_queue, const char *param)
    {
        if (param == nullptr)
            return;

        obs_x_.resize(n);
        obs_y_.resize(n);
        std::ifstream data(param);
        for (std::size_t i = 0; i != n; ++i)
            data >> obs_x_[i] >> obs_y_[i];
        data.close();

        command_queue.enqueue_write_buffer(
            dev_obs_x_, CL_TRUE, 0, sizeof(cl_float) * n, obs_x_.data());
        command_queue.enqueue_write_buffer(
            dev_obs_y_, CL_TRUE, 0, sizeof(cl_float) * n, obs_y_.data());
    }

    const vsmc::CLMemory &dev_rng_set() const { return dev_rng_set_; }
    const vsmc::CLMemory &dev_state() const { return dev_state_; }
    const vsmc::CLMemory &dev_obs_x() const { return dev_obs_x_; }
    const vsmc::CLMemory &dev_obs_y() const { return dev_obs_y_; }

    private:
    vsmc::CLMemory dev_rng_set_;
    vsmc::CLMemory dev_state_;
    vsmc::CLMemory dev_obs_x_;
    vsmc::CLMemory dev_obs_y_;
    vsmc::Vector<cl_float> obs_x_;
    vsmc::Vector<cl_float> obs_y_;
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
        particle.value().read_data(
            command_queue_, static_cast<const char *>(param));

        kernel_.set_arg(0, static_cast<cl_uint>(particle.size()));
        kernel_.set_arg(1, particle.value().dev_rng_set());
        kernel_.set_arg(2, particle.value().dev_state());
        kernel_.set_arg(3, particle.value().dev_obs_x());
        kernel_.set_arg(4, particle.value().dev_obs_y());

        particle.value().update_device(command_queue_);
        const std::size_t local = 256;
        const std::size_t global =
            particle.size() + local - particle.size() % local;
        command_queue_.enqueue_nd_range_kernel(kernel_, 1, vsmc::CLNDRange(),
            vsmc::CLNDRange(global), vsmc::CLNDRange(local));
        command_queue_.finish();
        particle.value().update_host(command_queue_);

        particle.weight().set_log(
            particle.value().data(), particle.value().dim());

        return 0;
    }

    private:
    vsmc::CLCommandQueue command_queue_;
    vsmc::CLKernel kernel_;
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
        kernel_.set_arg(0, static_cast<cl_uint>(t));
        kernel_.set_arg(1, static_cast<cl_uint>(particle.size()));
        kernel_.set_arg(2, particle.value().dev_rng_set());
        kernel_.set_arg(3, particle.value().dev_state());
        kernel_.set_arg(4, particle.value().dev_obs_x());
        kernel_.set_arg(5, particle.value().dev_obs_y());

        const std::size_t local = 256;
        const std::size_t global =
            particle.size() + local - particle.size() % local;
        particle.value().update_device(command_queue_);
        command_queue_.enqueue_nd_range_kernel(kernel_, 1, vsmc::CLNDRange(),
            vsmc::CLNDRange(global), vsmc::CLNDRange(local));
        command_queue_.finish();
        particle.value().update_host(command_queue_);

        particle.weight().add_log(
            particle.value().data(), particle.value().dim());

        return 0;
    }

    private:
    vsmc::CLCommandQueue command_queue_;
    vsmc::CLKernel kernel_;
};

class PFMEval : public vsmc::MonitorEvalTBB<PFState, PFMEval>
{
    public:
    void eval_sp(std::size_t t, std::size_t dim,
        vsmc::SingleParticle<PFState> sp, double *r)
    {
        r[0] = sp.state(1);
        r[1] = sp.state(2);
    }
};

int main()
{
    auto platform = vsmc::CLPlatform::platforms().front();
    auto device = platform.get_device(CL_DEVICE_TYPE_DEFAULT).front();
    cl_context_properties properties[] = {CL_CONTEXT_PLATFORM,
        reinterpret_cast<cl_context_properties>(platform.get()), 0};
    vsmc::CLContext context(properties, device);
    vsmc::CLCommandQueue command_queue(context, device, 0);

    std::ifstream source_cl("pf_ocl.cl");
    std::string source((std::istreambuf_iterator<char>(source_cl)),
        std::istreambuf_iterator<char>());
    source_cl.close();
    vsmc::CLProgram program(context, source);
    program.build(1, &device, "-I ../../include");

    vsmc::CLKernel kernel_init(program, "init");
    vsmc::CLKernel kernel_move(program, "move");

    vsmc::Sampler<PFState> sampler(N, vsmc::Multinomial, 0.5);
    sampler.init(PFInit(command_queue, kernel_init));
    sampler.move(PFMove(command_queue, kernel_move), false);
    sampler.monitor("pos", 2, PFMEval());
    sampler.particle().value().initialize(context);
    sampler.initialize(const_cast<char *>("pf.data")).iterate(n - 1);

    std::ofstream output("pf.out");
    output << sampler;
    output.close();

    return 0;
}
