#pragma once

#include <functional>

/**
 * This macro exists to make it easy to coerce member functions into c-style callbacks.
 *
 * Example:
 * // Use macro to create structs for coercion
 * C_CALLBACK(ConnectedCallback)
 *
 * // Assign member function to struct func
 * ConnectedCallback<void(PNIO_CBE_PRM*)>::func = [this](auto && arg) { Connected(std::forward<decltype(arg)>(arg)); };
 * RegisterCallback(PNIO_CBE_DEV_CONNECT_IND, ConnectedCallback<void(PNIO_CBE_PRM*)>::callback);
 *
 */
#define DECL_CALLBACK(name)                    \
  template <typename T>                        \
  struct name;                                 \
                                               \
  template <typename Ret, typename... Params>  \
  struct name<Ret(Params...)> {                \
    template <typename... Args>                \
    static Ret callback(Args... args) {        \
      return func(args...);                    \
    }                                          \
    static std::function<Ret(Params...)> func; \
  };                                           \
                                               \
  template <typename Ret, typename... Params>  \
  std::function<Ret(Params...)> name<Ret(Params...)>::func;

namespace controller::pn_driver {

DECL_CALLBACK(ConnectedCallback)
DECL_CALLBACK(DisconnectedCallback)
DECL_CALLBACK(OwnershipCallback)
DECL_CALLBACK(PrmEndCallback)
DECL_CALLBACK(InDataCallback)
DECL_CALLBACK(DiagnosticsCallback)
DECL_CALLBACK(ReadCallback)
DECL_CALLBACK(WriteCallback)
DECL_CALLBACK(EthernetCallback)

}  // namespace controller::pn_driver
