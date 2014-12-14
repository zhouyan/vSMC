//============================================================================
// vSMC/include/vsmc/opencl/backend_cl.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_OPENCL_BACKEND_CL_HPP
#define VSMC_OPENCL_BACKEND_CL_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/opencl/cl_buffer.hpp>
#include <vsmc/opencl/cl_configure.hpp>
#include <vsmc/opencl/cl_manager.hpp>
#include <vsmc/opencl/cl_manip.hpp>
#include <vsmc/opencl/internal/cl_copy.hpp>
#include <vsmc/rng/seed.hpp>
#include <vsmc/utility/array.hpp>

#define VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_DYNAMIC_STATE_SIZE_RESIZE(Dim) \
    VSMC_STATIC_ASSERT((Dim == Dynamic),                                     \
            USE_METHOD_resize_state_WITH_A_FIXED_SIZE_StateCL_OBJECT)

#define VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_TYPE(derived, user) \
    VSMC_STATIC_ASSERT((internal::IsDerivedFromStateCL<derived>::value),     \
            USE_##user##_WITH_A_STATE_TYPE_NOT_DERIVED_FROM_StateCL)

#define VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_FP_TYPE(type) \
    VSMC_STATIC_ASSERT((cxx11::is_same<type, ::cl_float>::value              \
                || cxx11::is_same<type, ::cl_double>::value),                \
            USE_StateCL_WITH_A_FP_TYPE_OTHER_THAN_cl_float_AND_cl_double)

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_BUILD(func) \
    VSMC_RUNTIME_ASSERT((build()),                                           \
            ("**StateCL::"#func"** CAN ONLY BE CALLED AFTER true "           \
             "**StateCL::build**"));

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_STATE_SIZE(state_size) \
    VSMC_RUNTIME_ASSERT((state_size >= 1), ("STATE SIZE IS LESS THAN 1"))

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_COPY_SIZE_MISMATCH \
    VSMC_RUNTIME_ASSERT((N == copy_.size()),                                 \
            ("**StateCL::copy** SIZE MISMATCH"))

#define VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_UNPACK_SIZE(psize, dim) \
    VSMC_RUNTIME_ASSERT((psize >= dim),                                      \
            ("**StateCL::state_unpack** INPUT PACK SIZE TOO SMALL"))

#if VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS

#define VSMC_DEFINE_OPENCL_COPY(Name) \
Name () : build_id_(-1) {}                                                   \
Name (const Name<T, PlaceHolder> &) = default;                               \
Name<T, PlaceHolder> &operator= (const Name<T, PlaceHolder> &) = default;    \
virtual ~Name () {}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
#define VSMC_DEFINE_OPENCL_MOVE(Name) \
Name (Name<T, PlaceHolder> &&) = default;                                    \
Name<T, PlaceHolder> &operator= (Name<T, PlaceHolder> &&) = default;
#else
#define VSMC_DEFINE_OPENCL_MOVE(Name)
#endif

#else // VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS

#define VSMC_DEFINE_OPENCL_COPY(Name) \
Name () : build_id_(-1) {}                                                   \
Name (const Name<T, PlaceHolder> &other) :                                   \
    configure_(other.configure_),                                            \
    build_id_(other.build_id_),                                              \
    kernel_(other.kernel_),                                                  \
    kernel_name_(other.kernel_name_) {}                                      \
Name<T, PlaceHolder> &operator= (const Name<T, PlaceHolder> &other)          \
{                                                                            \
    if (this != &other) {                                                    \
        configure_   = other.configure_;                                     \
        build_id_    = other.build_id_;                                      \
        kernel_      = other.kernel_;                                        \
        kernel_name_ = other.kernel_name_;                                   \
    }                                                                        \
}                                                                            \
virtual ~Name () {}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
#define VSMC_DEFINE_OPENCL_MOVE(Name) \
Name (Name<T, PlaceHolder> &&other) :                                        \
    configure_(cxx11::move(other.configure_)),                               \
    build_id_(other.build_id_),                                              \
    kernel_(cxx11::move(other.kernel_)),                                     \
    kernel_name_(cxx11::move(other.kernel_name_)) {}                         \
                                                                             \
Name<T, PlaceHolder> &operator= (Name<T, PlaceHolder> &&other)               \
{                                                                            \
    if (this != &other) {                                                    \
        configure_   = cxx11::move(other.configure_);                        \
        build_id_    = other.build_id_;                                      \
        kernel_      = cxx11::move(other.kernel_);                           \
        kernel_name_ = cxx11::move(other.kernel_name_);                      \
    }                                                                        \
                                                                             \
    return *this;                                                            \
}
#else
#define VSMC_DEFINE_OPENCL_MOVE(Name)
#endif

#endif // VSMC_HAS_CXX11_DEFAULTED_FUNCTIONS

#define VSMC_DEFINE_OPENCL_CONFIGURE_KERNEL \
CLConfigure &configure () {return configure_;}                               \
const CLConfigure &configure () const {return configure_;}                   \
::cl::Kernel &kernel () {return kernel_;}                                    \
const ::cl::Kernel &kernel () const {return kernel_;}                        \
const std::string &kernel_name () const {return kernel_name_;}

#define VSMC_DEFINE_OPENCL_SET_KERNEL \
if (kname.empty()) {                                                         \
    kernel_name_.clear();                                                    \
    return;                                                                  \
}                                                                            \
if (build_id_ != particle.value().build_id() || kernel_name_ != kname) {     \
    build_id_ = particle.value().build_id();                                 \
    kernel_name_ = kname;                                                    \
    kernel_ = particle.value().create_kernel(kernel_name_);                  \
    configure_.local_size(particle.size(),                                   \
            kernel_, particle.value().manager().device());                   \
}                                                                            \

#define VSMC_DEFINE_OPENCL_MEMBER_DATA \
CLConfigure configure_;                                                      \
int build_id_;                                                               \
::cl::Kernel kernel_;                                                        \
std::string kernel_name_

namespace vsmc {

namespace internal {

template <typename> void set_cl_fp_type (std::stringstream &);

template <>
inline void set_cl_fp_type<cl_float> (std::stringstream &ss)
{
    ss << "#ifndef FP_TYPE\n";
    ss << "#define FP_TYPE float\n";
    ss << "typedef float fp_type;\n";
    ss << "#endif\n";

    ss << "#ifndef VSMC_HAS_OPENCL_DOUBLE\n";
    ss << "#define VSMC_HAS_OPENCL_DOUBLE 0\n";
    ss << "#endif\n";
}

template <>
inline void set_cl_fp_type<cl_double> (std::stringstream &ss)
{
    ss << "#if defined(cl_khr_fp64)\n";
    ss << "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n";
    ss << "#elif defined(cl_amd_fp64)\n";
    ss << "#pragma OPENCL EXTENSION cl_amd_fp64 : enable\n";
    ss << "#endif\n";

    ss << "#define FP_TYPE double\n";
    ss << "typedef double fp_type;\n";
    ss << "#endif\n";

    ss << "#ifndef VSMC_HAS_OPENCL_DOUBLE\n";
    ss << "#define VSMC_HAS_OPENCL_DOUBLE 1\n";
    ss << "#endif\n";
}

template <typename FPType>
inline std::string cl_source_macros (
        std::size_t size, std::size_t state_size, std::size_t seed)
{
    std::stringstream ss;
    set_cl_fp_type<FPType>(ss);

    ss << "#ifndef SIZE\n";
    ss << "#define SIZE " << size << "UL\n";
    ss << "#endif\n";

    ss << "#ifndef STATE_SIZE\n";
    ss << "#define STATE_SIZE " << state_size << "UL\n";
    ss << "#endif\n";

    ss << "#ifndef SEED\n";
    ss << "#define SEED " << seed << "UL\n";
    ss << "#endif\n";

    return ss.str();
}

template <typename D>
struct IsDerivedFromStateCLImpl
{
    private :

    struct char2 {char c1; char c2;};

    template <std::size_t Dim, typename T, typename ID>
    static char test (const StateCL<Dim, T, ID> *);
    static char2 test (...);

    public :

   enum {value = sizeof(test(static_cast<const D *>(0))) == sizeof(char)};
}; // struct IsDerivedFromStateImpl

template <typename D>
struct IsDerivedFromStateCL :
    public cxx11::integral_constant<bool, IsDerivedFromStateCLImpl<D>::value>{};

} // namespace vsmc::internal

/// \brief Particle::value_type subtype using OpenCL
/// \ingroup OpenCL
template <std::size_t StateSize, typename FPType, typename ID>
class StateCL
{
    public :

    typedef ::cl_ulong size_type;
    typedef FPType fp_type;
    typedef ID cl_id;
    typedef CLManager<ID> manager_type;
    typedef typename cxx11::conditional<StateSize == Dynamic,
             std::vector<char>, Array<char, StateSize> >::type state_pack_type;

    explicit StateCL (size_type N) :
        state_size_(StateSize == Dynamic ? 1 : StateSize),
        size_(N), build_(false), build_id_(0),
        state_buffer_(state_size_ * size_)
        {
#if VSMC_OPENCL_VERSION >= 120
            if (manager().opencl_version() >= 120) {
                copy_from_buffer_.resize(size_,
                        CL_MEM_READ_ONLY|CL_MEM_HOST_WRITE_ONLY);
            } else {
                copy_from_buffer_.resize(size_, CL_MEM_READ_ONLY);
            }
#else
            copy_from_buffer_.resize(size_, CL_MEM_READ_ONLY);
#endif
        }

    size_type size () const {return size_;}

    std::size_t state_size () const {return state_size_;}

    /// \brief Change state size
    void resize_state (std::size_t state_size)
    {
        VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_DYNAMIC_STATE_SIZE_RESIZE(
                StateSize);
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_STATE_SIZE(state_size);

        state_size_ = state_size;
        state_buffer_.resize(state_size_ * size_);
    }

    /// \brief Change state buffer flag (cause reallocation)
    void update_state (::cl_mem_flags flag)
    {state_buffer_.resize(state_size_ * size_, flag);}

    /// \brief Change state buffer flag and host pointer (cause reallocation)
    void update_state (::cl_mem_flags flag, void *host_ptr)
    {state_buffer_.resize(state_size_ * size_, flag, host_ptr);}

    /// \brief The instance of the CLManager signleton associated with this
    /// value collcection
    static manager_type &manager () {return manager_type::instance();}

    /// \brief The OpenCL buffer that stores the state values
    const CLBuffer<char, ID> &state_buffer () const {return state_buffer_;}

    /// \brief The OpenCL program associated with this value collection
    const ::cl::Program &program () const {return program_;}

    /// \brief Build the OpenCL program from source
    ///
    /// \param source The source of the program
    /// \param flags The OpenCL compiler flags, e.g., `-I`
    /// \param os The output stream to write the output when error occurs
    ///
    /// \details
    /// Note that a few macros are defined before the user supplied `source`.
    /// Say the template parameter `StateSize == 4`, `FPType` of this class is
    /// set to `cl_float`, and there are `1000` particles, then the complete
    /// source, which acutally get compiled looks like the following
    /// ~~~{.cpp}
    /// #ifndef FP_TYPE
    /// #define FP_TYPE float
    /// typedef float fp_type;
    /// #endif
    ///
    /// #ifndef VSMC_HAS_OPENCL_DOUBLE
    /// #define VSMC_HAS_OPENCL_DOUBLE 0
    /// #endif
    ///
    /// #ifndef SIZE
    /// #define SIZE 1000UL;
    /// #endif
    ///
    /// #ifndef STATE_SIZE
    /// #define STATE_SIZE 4UL;
    /// #endif
    ///
    /// #ifndef SEED
    /// #define SEED 101UL;
    /// #endif
    /// // The actual seed is vsmc::Seed::instance().get()
    /// // ... User source, passed by the source argument
    /// ~~~
    /// After build, `vsmc::Seed::instance().skip(N)` is called with `N`
    /// being the nubmer of particles.
    template <typename CharT, typename Traits>
    void build (const std::string &source,
            const std::string &flags, std::basic_ostream<CharT, Traits> &os)
    {
        VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_FP_TYPE(fp_type);

        std::string src(internal::cl_source_macros<fp_type>(
                    size_, state_size_, Seed::instance().get()) + source);
        Seed::instance().skip(static_cast<Seed::skip_type>(size_));
        program_ = manager().create_program(src);
        build_program(flags, os);
    }

    void build (const std::string &source,
            const std::string &flags = std::string())
    {build(source, flags, std::cout);}

    /// \brief Build from an existing program
    template <typename CharT, typename Traits>
    void build (const ::cl::Program &program,
            const std::string &flags, std::basic_ostream<CharT, Traits> &os)
    {
        program_ = program;
        build_program(flags, os);
    }

    void build (const ::cl::Program &program,
            const std::string &flags = std::string())
    {build(program, flags, std::cout);}

    /// \brief Whether the last attempted building success
    bool build () const {return build_;}

    /// \brief The build id of the last attempted of building
    ///
    /// \details
    /// This function returns a non-decreasing sequence of integers
    int build_id () const {return build_id_;}

    /// \brief Create kernel with the current program
    ///
    /// \details
    /// If build() does not return `true`, then calling this is an error
    ::cl::Kernel create_kernel (const std::string &name) const
    {
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_BUILD(create_kernel);

        return ::cl::Kernel(program_, name.c_str());
    }

    template <typename IntType>
    void copy (std::size_t N, const IntType *copy_from)
    {
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_COPY_SIZE_MISMATCH;

        manager().template write_buffer<size_type>(
                copy_from_buffer_.data(), N, copy_from);
        copy_(copy_from_buffer_.data(), state_buffer_.data());
    }

    void copy_pre_processor ()
    {
        state_idx_host_.resize(size_);
#if VSMC_OPENCL_VERSION >= 120
        if (manager().opencl_version() >= 120) {
            state_idx_buffer_.resize(size_,
                    CL_MEM_READ_ONLY|CL_MEM_HOST_WRITE_ONLY|
                    CL_MEM_USE_HOST_PTR, &state_idx_host_[0]);
        } else {
            state_idx_buffer_.resize(size_,
                    CL_MEM_READ_ONLY|CL_MEM_USE_HOST_PTR, &state_idx_host_[0]);
        }
#else
        state_idx_buffer_.resize(size_,
                CL_MEM_READ_ONLY|CL_MEM_USE_HOST_PTR, &state_idx_host_[0]);
#endif

        state_tmp_host_.resize(size_ * state_size_);
        state_tmp_buffer_.resize(size_ * state_size_,
                CL_MEM_READ_ONLY|CL_MEM_USE_HOST_PTR, &state_tmp_host_[0]);

        std::memset(&state_idx_host_[0], 0, size_);
        manager().read_buffer(state_tmp_buffer_.data(), size_ * state_size_,
                &state_tmp_host_[0]);
    }

    void copy_post_processor ()
    {
        manager().write_buffer(state_idx_buffer_.data(), size_,
                &state_idx_host_[0]);
        manager().write_buffer(state_tmp_buffer_.data(), size_ * state_size_,
                &state_tmp_host_[0]);
        copy_(state_idx_buffer_.data(), state_tmp_buffer_.data(),
                state_buffer_.data());
    }

    state_pack_type state_pack (size_type id) const
    {
        state_pack_type pack(create_pack());
        std::memcpy(&pack[0], &state_tmp_host_[id * state_size_], state_size_);

        return pack;
    }

    void state_unpack (size_type id, const state_pack_type &pack)
    {
        VSMC_RUNTIME_ASSERT_OPENCL_BACKEND_CL_UNPACK_SIZE(
                pack.size(), state_size_);

        state_idx_host_[id] = 1;
        std::memcpy(&state_tmp_host_[id * state_size_], &pack[0], state_size_);
    }

    CLConfigure &copy_configure () {return copy_.configure();}

    const CLConfigure &copy_configure () const {return copy_.configure();}

    ::cl::Kernel &copy_kernel () {return copy_.kernel();}

    const ::cl::Kernel &copy_kernel () const {return copy_.kernel();}

    private :

    std::size_t state_size_;
    size_type size_;

    ::cl::Program program_;

    bool build_;
    int build_id_;

    CLBuffer<char, ID> state_buffer_;
    CLBuffer<size_type, ID> copy_from_buffer_;
    internal::CLCopy<ID> copy_;

    CLBuffer<char, ID> state_idx_buffer_;
    CLBuffer<char, ID> state_tmp_buffer_;
    std::vector<char, AlignedAllocator<char> > state_idx_host_;
    std::vector<char, AlignedAllocator<char> > state_tmp_host_;

    template <typename CharT, typename Traits>
    void build_program (const std::string flags,
            std::basic_ostream<CharT, Traits> &os)
    {
        ++build_id_;

        build_ = false;
        try {
            program_.build(manager().device_vec(), flags.c_str());
            copy_.build(size_, state_size_);
            build_ = true;
        } catch (...) {
            CLQuery::program_build_log(program_, os);
            CLQuery::program_build_log(copy_.program(), os);
            throw;
        }
    }

    state_pack_type create_pack () const
    {
        return create_pack_dispatch(
                cxx11::integral_constant<bool, StateSize == Dynamic>());
    }

    std::vector<char> create_pack_dispatch (cxx11::true_type) const
    {return std::vector<char>(this->state_size());}

    Array<char, StateSize> create_pack_dispatch (cxx11::false_type) const
    {return Array<char, StateSize>();}
}; // class StateCL

/// \brief Sampler<T>::init_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`initialize_state`)
/// ~~~{.cpp}
/// __kernel
/// void kern (__global state_type *state, __global ulong *accept);
/// ~~~
/// - Kernels can have additonal arguments and set by the user in
/// `pre_processor`.
/// - `state` has size `N * StateSize` where `accept` has size `N`.
/// - The declaration does not have to much this, but the first arguments will
/// be set by InitializeCL::opeartor(). For example
/// ~~~{.cpp}
/// type struct {
///     state_type v1;
///     state_type v2;
///     // ...
///     state_type vDim; // vDim = StateSize / state_type
/// } param;
/// __kernel
/// void kern (__global param *state, __global ulong *accept);
/// ~~~
/// is also acceptable, but now `state` has to be treat as a length `N` array.
/// In summary, on the host side, it is a `cl::Buffer` object being passed to
/// the kernel, which is not much unlike `void *` pointer.
template <typename T, typename PlaceHolder = NullType>
class InitializeCL
{
    public :

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    static VSMC_CONSTEXPR ::cl_uint kernel_args_offset () {return 2;}

    std::size_t operator() (Particle<T> &particle, void *param)
    {
        VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_TYPE(T, InitializeCL);

        set_kernel(particle);
        if (kernel_name_.empty())
            return 0;

        set_kernel_args(particle);
        initialize_param(particle, param);
        pre_processor(particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        post_processor(particle);

        return accept_count(particle, accept_buffer_.data());
    }

    virtual void initialize_param (Particle<T> &, void *) {}
    virtual void initialize_state (std::string &) {}
    virtual void pre_processor (Particle<T> &) {}
    virtual void post_processor (Particle<T> &) {}

    virtual std::size_t accept_count (Particle<T> &particle,
            const ::cl::Buffer &accept_buffer)
    {
        particle.value().manager().read_buffer(
                accept_buffer, particle.size(), &accept_host_[0]);

        return static_cast<std::size_t>(std::accumulate(
                    accept_host_.begin(), accept_host_.end(),
                    static_cast< ::cl_ulong>(0)));
    }

    virtual void set_kernel (const Particle<T> &particle)
    {
        std::string kname;
        initialize_state(kname);
        VSMC_DEFINE_OPENCL_SET_KERNEL;
    }

    virtual void set_kernel_args (const Particle<T> &particle)
    {
        accept_host_.resize(particle.size());
#if VSMC_OPENCL_VERSION >= 120
        if (particle.value().manager().opencl_version() >= 120) {
            accept_buffer_.resize(particle.size(),
                    CL_MEM_READ_WRITE|CL_MEM_HOST_READ_ONLY|
                    CL_MEM_USE_HOST_PTR, &accept_host_[0]);
        } else {
            accept_buffer_.resize(particle.size(),
                    CL_MEM_READ_WRITE|CL_MEM_USE_HOST_PTR, &accept_host_[0]);
        }
#else
        accept_buffer_.resize(particle.size(),
                CL_MEM_READ_WRITE|CL_MEM_USE_HOST_PTR, &accept_host_[0]);
#endif
        cl_set_kernel_args(kernel_, 0,
                particle.value().state_buffer().data(), accept_buffer_.data());
    }

    VSMC_DEFINE_OPENCL_CONFIGURE_KERNEL

    protected :

    VSMC_DEFINE_OPENCL_COPY(InitializeCL)
    VSMC_DEFINE_OPENCL_MOVE(InitializeCL)

    private :

    VSMC_DEFINE_OPENCL_MEMBER_DATA;
    CLBuffer< ::cl_ulong, typename T::cl_id> accept_buffer_;
    std::vector< ::cl_ulong> accept_host_;
}; // class InitializeCL

/// \brief Sampler<T>::move_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`move_state`)
/// ~~~{.cpp}
/// __kernel
/// void kern (ulong iter, __global state_type *state, __global ulong *accept);
/// ~~~
template <typename T, typename PlaceHolder = NullType>
class MoveCL
{
    public :

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    static VSMC_CONSTEXPR ::cl_uint kernel_args_offset () {return 3;}

    std::size_t operator() (std::size_t iter, Particle<T> &particle)
    {
        VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_TYPE(T, MoveCL);

        set_kernel(iter, particle);
        if (kernel_name_.empty())
            return 0;

        set_kernel_args(iter, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        post_processor(iter, particle);

        return accept_count(particle, accept_buffer_.data());
    }

    virtual void move_state (std::size_t, std::string &) {}
    virtual void pre_processor (std::size_t, Particle<T> &) {}
    virtual void post_processor (std::size_t, Particle<T> &) {}

    virtual std::size_t accept_count (Particle<T> &particle,
            const ::cl::Buffer &accept_buffer)
    {
        particle.value().manager().read_buffer(
                accept_buffer, particle.size(), &accept_host_[0]);

        return static_cast<std::size_t>(std::accumulate(
                    accept_host_.begin(), accept_host_.end(),
                    static_cast< ::cl_ulong>(0)));
    }

    virtual void set_kernel (std::size_t iter, const Particle<T> &particle)
    {
        std::string kname;
        move_state(iter, kname);
        VSMC_DEFINE_OPENCL_SET_KERNEL;
    }

    virtual void set_kernel_args (std::size_t iter,
            const Particle<T> &particle)
    {
        accept_host_.resize(particle.size());
#if VSMC_OPENCL_VERSION >= 120
        if (particle.value().manager().opencl_version() >= 120) {
            accept_buffer_.resize(particle.size(),
                    CL_MEM_READ_WRITE|CL_MEM_HOST_READ_ONLY|
                    CL_MEM_USE_HOST_PTR, &accept_host_[0]);
        } else {
            accept_buffer_.resize(particle.size(),
                    CL_MEM_READ_WRITE|CL_MEM_USE_HOST_PTR, &accept_host_[0]);
        }
#else
        accept_buffer_.resize(particle.size(),
                CL_MEM_READ_WRITE|CL_MEM_USE_HOST_PTR, &accept_host_[0]);
#endif
        cl_set_kernel_args(kernel_, 0, static_cast< ::cl_ulong>(iter),
                particle.value().state_buffer().data(), accept_buffer_.data());
    }

    VSMC_DEFINE_OPENCL_CONFIGURE_KERNEL

    protected :

    VSMC_DEFINE_OPENCL_COPY(MoveCL)
    VSMC_DEFINE_OPENCL_MOVE(MoveCL)

    private :

    VSMC_DEFINE_OPENCL_MEMBER_DATA;
    CLBuffer< ::cl_ulong, typename T::cl_id> accept_buffer_;
    std::vector< ::cl_ulong> accept_host_;
}; // class MoveCL

/// \brief Monitor<T>::eval_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`monitor_state`)
/// ~~~{.cpp}
/// __kernel
/// void kern (ulong iter, ulong dim, __global state_type *state,
///            __global fp_type *res);
/// ~~~
template <typename T, typename PlaceHolder = NullType>
class MonitorEvalCL
{
    public :

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    static VSMC_CONSTEXPR ::cl_uint kernel_args_offset () {return 4;}

    void operator() (std::size_t iter, std::size_t dim,
            const Particle<T> &particle, double *res)
    {
        VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_TYPE(T, MonitorEvalCL);

        set_kernel(iter, dim, particle);
        if (kernel_name_.empty())
            return;

        set_kernel_args(iter, dim, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        particle.value().manager().template
            read_buffer<typename T::fp_type>(
                    buffer_.data(), particle.value().size() * dim, res);
        post_processor(iter, particle);
    }

    virtual void monitor_state (std::size_t, std::string &) {}
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    virtual void set_kernel (std::size_t iter, std::size_t,
            const Particle<T> &particle)
    {
        std::string kname;
        monitor_state(iter, kname);
        VSMC_DEFINE_OPENCL_SET_KERNEL;
    }

    virtual void set_kernel_args (std::size_t iter, std::size_t dim,
            const Particle<T> &particle)
    {
#if VSMC_OPENCL_VERSION >= 120
        if (particle.value().manager().opencl_version() >= 120) {
            buffer_.resize(particle.size() * dim,
                    CL_MEM_READ_WRITE|CL_MEM_HOST_READ_ONLY);
        } else {
            buffer_.resize(particle.size() * dim);
        }
#else
        buffer_.resize(particle.size() * dim);
#endif
        cl_set_kernel_args(kernel_, 0, static_cast< ::cl_ulong>(iter),
                static_cast< ::cl_ulong>(dim),
                particle.value().state_buffer().data(), buffer_.data());
    }

    VSMC_DEFINE_OPENCL_CONFIGURE_KERNEL

    protected :

    VSMC_DEFINE_OPENCL_COPY(MonitorEvalCL)
    VSMC_DEFINE_OPENCL_MOVE(MonitorEvalCL)

    private :

    VSMC_DEFINE_OPENCL_MEMBER_DATA;
    CLBuffer<typename T::fp_type, typename T::cl_id> buffer_;
}; // class MonitorEvalCL

/// \brief Path<T>::eval_type subtype using OpenCL
/// \ingroup OpenCL
///
/// \details
/// Kernel requirement (`path_state`)
/// ~~~{.cpp}
/// __kernel
/// void kern (ulong iter, __global state_type *state,
///            __global state_type *res);
/// ~~~
template <typename T, typename PlaceHolder = NullType>
class PathEvalCL
{
    public :

    /// \brief The index offset of additional kernel arguments set by the user
    ///
    /// \details
    /// The first user supplied additional argument shall have index
    /// `kernel_args_offset`
    static VSMC_CONSTEXPR ::cl_uint kernel_args_offset () {return 3;}

    double operator() (std::size_t iter, const Particle<T> &particle,
        double *res)
    {
        VSMC_STATIC_ASSERT_OPENCL_BACKEND_CL_STATE_CL_TYPE(T, PathEvalCL);

        set_kernel(iter, particle);
        if (kernel_name_.empty())
            return 0;

        set_kernel_args(iter, particle);
        pre_processor(iter, particle);
        particle.value().manager().run_kernel(
                kernel_, particle.size(), configure_.local_size());
        particle.value().manager().template
            read_buffer<typename T::fp_type>(
                    buffer_.data(), particle.value().size(), res);
        post_processor(iter, particle);

        return this->path_grid(iter, particle);
    }

    virtual void path_state (std::size_t, std::string &) {}
    virtual double path_grid (std::size_t, const Particle<T> &) {return 0;}
    virtual void pre_processor (std::size_t, const Particle<T> &) {}
    virtual void post_processor (std::size_t, const Particle<T> &) {}

    virtual void set_kernel (std::size_t iter, const Particle<T> &particle)
    {
        std::string kname;
        path_state(iter, kname);
        VSMC_DEFINE_OPENCL_SET_KERNEL;
    }

    virtual void set_kernel_args (std::size_t iter,
            const Particle<T> &particle)
    {
#if VSMC_OPENCL_VERSION >= 120
        if (particle.value().manager().opencl_version() >= 120) {
            buffer_.resize(particle.size(),
                    CL_MEM_READ_WRITE|CL_MEM_HOST_READ_ONLY);
        } else {
            buffer_.resize(particle.size());
        }
#else
        buffer_.resize(particle.size());
#endif
        cl_set_kernel_args(kernel_, 0, static_cast< ::cl_ulong>(iter),
                particle.value().state_buffer().data(), buffer_.data());
    }

    VSMC_DEFINE_OPENCL_CONFIGURE_KERNEL

    protected :

    VSMC_DEFINE_OPENCL_COPY(PathEvalCL)
    VSMC_DEFINE_OPENCL_MOVE(PathEvalCL)

    private :

    VSMC_DEFINE_OPENCL_MEMBER_DATA;
    CLBuffer<typename T::fp_type, typename T::cl_id> buffer_;
}; // class PathEvalCL

} // namespace vsmc

#endif // VSMC_OPENCL_BACKEND_CL_HPP
