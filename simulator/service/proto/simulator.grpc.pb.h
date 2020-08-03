// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: simulator.proto
#ifndef GRPC_simulator_2eproto__INCLUDED
#define GRPC_simulator_2eproto__INCLUDED

#include "simulator.pb.h"

#include <functional>
#include <grpc/impl/codegen/port_platform.h>
#include <grpcpp/impl/codegen/async_generic_service.h>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/client_context.h>
#include <grpcpp/impl/codegen/completion_queue.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/proto_utils.h>
#include <grpcpp/impl/codegen/rpc_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/status.h>
#include <grpcpp/impl/codegen/stub_options.h>
#include <grpcpp/impl/codegen/sync_stream.h>

namespace service {

// The simulator service definition.
class SimulatorServer final {
 public:
  static constexpr char const* service_full_name() {
    return "service.SimulatorServer";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // fetch environment from server
    virtual ::grpc::Status FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::service::FetchEnvResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::FetchEnvResponse>> AsyncFetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::FetchEnvResponse>>(AsyncFetchEnvRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::FetchEnvResponse>> PrepareAsyncFetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::FetchEnvResponse>>(PrepareAsyncFetchEnvRaw(context, request, cq));
    }
    // push my status to server
    virtual ::grpc::Status PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::service::PushMyTrajectoryResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::PushMyTrajectoryResponse>> AsyncPushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::PushMyTrajectoryResponse>>(AsyncPushMyTrajectoryRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::PushMyTrajectoryResponse>> PrepareAsyncPushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::service::PushMyTrajectoryResponse>>(PrepareAsyncPushMyTrajectoryRaw(context, request, cq));
    }
    class experimental_async_interface {
     public:
      virtual ~experimental_async_interface() {}
      // fetch environment from server
      virtual void FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void FetchEnv(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::FetchEnvResponse* response, std::function<void(::grpc::Status)>) = 0;
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      virtual void FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      #else
      virtual void FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) = 0;
      #endif
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      virtual void FetchEnv(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::FetchEnvResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      #else
      virtual void FetchEnv(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::FetchEnvResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) = 0;
      #endif
      // push my status to server
      virtual void PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void PushMyTrajectory(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::PushMyTrajectoryResponse* response, std::function<void(::grpc::Status)>) = 0;
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      virtual void PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      #else
      virtual void PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) = 0;
      #endif
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      virtual void PushMyTrajectory(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::PushMyTrajectoryResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      #else
      virtual void PushMyTrajectory(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::PushMyTrajectoryResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) = 0;
      #endif
    };
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    typedef class experimental_async_interface async_interface;
    #endif
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    async_interface* async() { return experimental_async(); }
    #endif
    virtual class experimental_async_interface* experimental_async() { return nullptr; }
  private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::service::FetchEnvResponse>* AsyncFetchEnvRaw(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::service::FetchEnvResponse>* PrepareAsyncFetchEnvRaw(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::service::PushMyTrajectoryResponse>* AsyncPushMyTrajectoryRaw(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::service::PushMyTrajectoryResponse>* PrepareAsyncPushMyTrajectoryRaw(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    ::grpc::Status FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::service::FetchEnvResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::FetchEnvResponse>> AsyncFetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::FetchEnvResponse>>(AsyncFetchEnvRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::FetchEnvResponse>> PrepareAsyncFetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::FetchEnvResponse>>(PrepareAsyncFetchEnvRaw(context, request, cq));
    }
    ::grpc::Status PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::service::PushMyTrajectoryResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::PushMyTrajectoryResponse>> AsyncPushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::PushMyTrajectoryResponse>>(AsyncPushMyTrajectoryRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::PushMyTrajectoryResponse>> PrepareAsyncPushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::service::PushMyTrajectoryResponse>>(PrepareAsyncPushMyTrajectoryRaw(context, request, cq));
    }
    class experimental_async final :
      public StubInterface::experimental_async_interface {
     public:
      void FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response, std::function<void(::grpc::Status)>) override;
      void FetchEnv(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::FetchEnvResponse* response, std::function<void(::grpc::Status)>) override;
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      void FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      #else
      void FetchEnv(::grpc::ClientContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) override;
      #endif
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      void FetchEnv(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::FetchEnvResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      #else
      void FetchEnv(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::FetchEnvResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) override;
      #endif
      void PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response, std::function<void(::grpc::Status)>) override;
      void PushMyTrajectory(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::PushMyTrajectoryResponse* response, std::function<void(::grpc::Status)>) override;
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      void PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      #else
      void PushMyTrajectory(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) override;
      #endif
      #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      void PushMyTrajectory(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::PushMyTrajectoryResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      #else
      void PushMyTrajectory(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::service::PushMyTrajectoryResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) override;
      #endif
     private:
      friend class Stub;
      explicit experimental_async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class experimental_async_interface* experimental_async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class experimental_async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::service::FetchEnvResponse>* AsyncFetchEnvRaw(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::service::FetchEnvResponse>* PrepareAsyncFetchEnvRaw(::grpc::ClientContext* context, const ::service::FetchEnvRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::service::PushMyTrajectoryResponse>* AsyncPushMyTrajectoryRaw(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::service::PushMyTrajectoryResponse>* PrepareAsyncPushMyTrajectoryRaw(::grpc::ClientContext* context, const ::service::PushMyTrajectoryRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_FetchEnv_;
    const ::grpc::internal::RpcMethod rpcmethod_PushMyTrajectory_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // fetch environment from server
    virtual ::grpc::Status FetchEnv(::grpc::ServerContext* context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response);
    // push my status to server
    virtual ::grpc::Status PushMyTrajectory(::grpc::ServerContext* context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_FetchEnv : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_FetchEnv() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_FetchEnv() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status FetchEnv(::grpc::ServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestFetchEnv(::grpc::ServerContext* context, ::service::FetchEnvRequest* request, ::grpc::ServerAsyncResponseWriter< ::service::FetchEnvResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_PushMyTrajectory : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_PushMyTrajectory() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_PushMyTrajectory() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status PushMyTrajectory(::grpc::ServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestPushMyTrajectory(::grpc::ServerContext* context, ::service::PushMyTrajectoryRequest* request, ::grpc::ServerAsyncResponseWriter< ::service::PushMyTrajectoryResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_FetchEnv<WithAsyncMethod_PushMyTrajectory<Service > > AsyncService;
  template <class BaseClass>
  class ExperimentalWithCallbackMethod_FetchEnv : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithCallbackMethod_FetchEnv() {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::Service::
    #else
      ::grpc::Service::experimental().
    #endif
        MarkMethodCallback(0,
          new ::grpc_impl::internal::CallbackUnaryHandler< ::service::FetchEnvRequest, ::service::FetchEnvResponse>(
            [this](
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
                   ::grpc::CallbackServerContext*
    #else
                   ::grpc::experimental::CallbackServerContext*
    #endif
                     context, const ::service::FetchEnvRequest* request, ::service::FetchEnvResponse* response) { return this->FetchEnv(context, request, response); }));}
    void SetMessageAllocatorFor_FetchEnv(
        ::grpc::experimental::MessageAllocator< ::service::FetchEnvRequest, ::service::FetchEnvResponse>* allocator) {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
    #else
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::experimental().GetHandler(0);
    #endif
      static_cast<::grpc_impl::internal::CallbackUnaryHandler< ::service::FetchEnvRequest, ::service::FetchEnvResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~ExperimentalWithCallbackMethod_FetchEnv() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status FetchEnv(::grpc::ServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    virtual ::grpc::ServerUnaryReactor* FetchEnv(
      ::grpc::CallbackServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/)
    #else
    virtual ::grpc::experimental::ServerUnaryReactor* FetchEnv(
      ::grpc::experimental::CallbackServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/)
    #endif
      { return nullptr; }
  };
  template <class BaseClass>
  class ExperimentalWithCallbackMethod_PushMyTrajectory : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithCallbackMethod_PushMyTrajectory() {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::Service::
    #else
      ::grpc::Service::experimental().
    #endif
        MarkMethodCallback(1,
          new ::grpc_impl::internal::CallbackUnaryHandler< ::service::PushMyTrajectoryRequest, ::service::PushMyTrajectoryResponse>(
            [this](
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
                   ::grpc::CallbackServerContext*
    #else
                   ::grpc::experimental::CallbackServerContext*
    #endif
                     context, const ::service::PushMyTrajectoryRequest* request, ::service::PushMyTrajectoryResponse* response) { return this->PushMyTrajectory(context, request, response); }));}
    void SetMessageAllocatorFor_PushMyTrajectory(
        ::grpc::experimental::MessageAllocator< ::service::PushMyTrajectoryRequest, ::service::PushMyTrajectoryResponse>* allocator) {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
    #else
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::experimental().GetHandler(1);
    #endif
      static_cast<::grpc_impl::internal::CallbackUnaryHandler< ::service::PushMyTrajectoryRequest, ::service::PushMyTrajectoryResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~ExperimentalWithCallbackMethod_PushMyTrajectory() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status PushMyTrajectory(::grpc::ServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    virtual ::grpc::ServerUnaryReactor* PushMyTrajectory(
      ::grpc::CallbackServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/)
    #else
    virtual ::grpc::experimental::ServerUnaryReactor* PushMyTrajectory(
      ::grpc::experimental::CallbackServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/)
    #endif
      { return nullptr; }
  };
  #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
  typedef ExperimentalWithCallbackMethod_FetchEnv<ExperimentalWithCallbackMethod_PushMyTrajectory<Service > > CallbackService;
  #endif

  typedef ExperimentalWithCallbackMethod_FetchEnv<ExperimentalWithCallbackMethod_PushMyTrajectory<Service > > ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_FetchEnv : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_FetchEnv() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_FetchEnv() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status FetchEnv(::grpc::ServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_PushMyTrajectory : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_PushMyTrajectory() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_PushMyTrajectory() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status PushMyTrajectory(::grpc::ServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_FetchEnv : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_FetchEnv() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_FetchEnv() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status FetchEnv(::grpc::ServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestFetchEnv(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_PushMyTrajectory : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_PushMyTrajectory() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_PushMyTrajectory() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status PushMyTrajectory(::grpc::ServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestPushMyTrajectory(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class ExperimentalWithRawCallbackMethod_FetchEnv : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithRawCallbackMethod_FetchEnv() {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::Service::
    #else
      ::grpc::Service::experimental().
    #endif
        MarkMethodRawCallback(0,
          new ::grpc_impl::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
                   ::grpc::CallbackServerContext*
    #else
                   ::grpc::experimental::CallbackServerContext*
    #endif
                     context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->FetchEnv(context, request, response); }));
    }
    ~ExperimentalWithRawCallbackMethod_FetchEnv() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status FetchEnv(::grpc::ServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    virtual ::grpc::ServerUnaryReactor* FetchEnv(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)
    #else
    virtual ::grpc::experimental::ServerUnaryReactor* FetchEnv(
      ::grpc::experimental::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)
    #endif
      { return nullptr; }
  };
  template <class BaseClass>
  class ExperimentalWithRawCallbackMethod_PushMyTrajectory : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    ExperimentalWithRawCallbackMethod_PushMyTrajectory() {
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
      ::grpc::Service::
    #else
      ::grpc::Service::experimental().
    #endif
        MarkMethodRawCallback(1,
          new ::grpc_impl::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
                   ::grpc::CallbackServerContext*
    #else
                   ::grpc::experimental::CallbackServerContext*
    #endif
                     context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->PushMyTrajectory(context, request, response); }));
    }
    ~ExperimentalWithRawCallbackMethod_PushMyTrajectory() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status PushMyTrajectory(::grpc::ServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    #ifdef GRPC_CALLBACK_API_NONEXPERIMENTAL
    virtual ::grpc::ServerUnaryReactor* PushMyTrajectory(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)
    #else
    virtual ::grpc::experimental::ServerUnaryReactor* PushMyTrajectory(
      ::grpc::experimental::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)
    #endif
      { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_FetchEnv : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_FetchEnv() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler< ::service::FetchEnvRequest, ::service::FetchEnvResponse>(std::bind(&WithStreamedUnaryMethod_FetchEnv<BaseClass>::StreamedFetchEnv, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_FetchEnv() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status FetchEnv(::grpc::ServerContext* /*context*/, const ::service::FetchEnvRequest* /*request*/, ::service::FetchEnvResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedFetchEnv(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::service::FetchEnvRequest,::service::FetchEnvResponse>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_PushMyTrajectory : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_PushMyTrajectory() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler< ::service::PushMyTrajectoryRequest, ::service::PushMyTrajectoryResponse>(std::bind(&WithStreamedUnaryMethod_PushMyTrajectory<BaseClass>::StreamedPushMyTrajectory, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_PushMyTrajectory() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status PushMyTrajectory(::grpc::ServerContext* /*context*/, const ::service::PushMyTrajectoryRequest* /*request*/, ::service::PushMyTrajectoryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedPushMyTrajectory(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::service::PushMyTrajectoryRequest,::service::PushMyTrajectoryResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_FetchEnv<WithStreamedUnaryMethod_PushMyTrajectory<Service > > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_FetchEnv<WithStreamedUnaryMethod_PushMyTrajectory<Service > > StreamedService;
};

}  // namespace service


#endif  // GRPC_simulator_2eproto__INCLUDED