// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: rest_response.proto

#ifndef PROTOBUF_rest_5fresponse_2eproto__INCLUDED
#define PROTOBUF_rest_5fresponse_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_rest_5fresponse_2eproto();
void protobuf_AssignDesc_rest_5fresponse_2eproto();
void protobuf_ShutdownFile_rest_5fresponse_2eproto();

class RestResponse;

enum RestResponse_Type {
  RestResponse_Type_SUCCESS = 1,
  RestResponse_Type_ERROR = 2,
  RestResponse_Type_LOGIN = 3,
  RestResponse_Type_LOGOUT = 4
};
GZ_MSGS_VISIBLE bool RestResponse_Type_IsValid(int value);
const RestResponse_Type RestResponse_Type_Type_MIN = RestResponse_Type_SUCCESS;
const RestResponse_Type RestResponse_Type_Type_MAX = RestResponse_Type_LOGOUT;
const int RestResponse_Type_Type_ARRAYSIZE = RestResponse_Type_Type_MAX + 1;

GZ_MSGS_VISIBLE const ::google::protobuf::EnumDescriptor* RestResponse_Type_descriptor();
inline const ::std::string& RestResponse_Type_Name(RestResponse_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    RestResponse_Type_descriptor(), value);
}
inline bool RestResponse_Type_Parse(
    const ::std::string& name, RestResponse_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<RestResponse_Type>(
    RestResponse_Type_descriptor(), name, value);
}
// ===================================================================

class GZ_MSGS_VISIBLE RestResponse : public ::google::protobuf::Message {
 public:
  RestResponse();
  virtual ~RestResponse();

  RestResponse(const RestResponse& from);

  inline RestResponse& operator=(const RestResponse& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const RestResponse& default_instance();

  void Swap(RestResponse* other);

  // implements Message ----------------------------------------------

  RestResponse* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RestResponse& from);
  void MergeFrom(const RestResponse& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef RestResponse_Type Type;
  static const Type SUCCESS = RestResponse_Type_SUCCESS;
  static const Type ERROR = RestResponse_Type_ERROR;
  static const Type LOGIN = RestResponse_Type_LOGIN;
  static const Type LOGOUT = RestResponse_Type_LOGOUT;
  static inline bool Type_IsValid(int value) {
    return RestResponse_Type_IsValid(value);
  }
  static const Type Type_MIN =
    RestResponse_Type_Type_MIN;
  static const Type Type_MAX =
    RestResponse_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    RestResponse_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return RestResponse_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return RestResponse_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return RestResponse_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional uint32 id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline ::google::protobuf::uint32 id() const;
  inline void set_id(::google::protobuf::uint32 value);

  // required .gazebo.msgs.RestResponse.Type type = 2;
  inline bool has_type() const;
  inline void clear_type();
  static const int kTypeFieldNumber = 2;
  inline ::gazebo::msgs::RestResponse_Type type() const;
  inline void set_type(::gazebo::msgs::RestResponse_Type value);

  // optional string msg = 3;
  inline bool has_msg() const;
  inline void clear_msg();
  static const int kMsgFieldNumber = 3;
  inline const ::std::string& msg() const;
  inline void set_msg(const ::std::string& value);
  inline void set_msg(const char* value);
  inline void set_msg(const char* value, size_t size);
  inline ::std::string* mutable_msg();
  inline ::std::string* release_msg();
  inline void set_allocated_msg(::std::string* msg);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.RestResponse)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_msg();
  inline void clear_has_msg();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 id_;
  int type_;
  ::std::string* msg_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_rest_5fresponse_2eproto();
  friend void protobuf_AssignDesc_rest_5fresponse_2eproto();
  friend void protobuf_ShutdownFile_rest_5fresponse_2eproto();

  void InitAsDefaultInstance();
  static RestResponse* default_instance_;
};
// ===================================================================


// ===================================================================

// RestResponse

// optional uint32 id = 1;
inline bool RestResponse::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RestResponse::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RestResponse::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RestResponse::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 RestResponse::id() const {
  return id_;
}
inline void RestResponse::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
}

// required .gazebo.msgs.RestResponse.Type type = 2;
inline bool RestResponse::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RestResponse::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RestResponse::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void RestResponse::clear_type() {
  type_ = 1;
  clear_has_type();
}
inline ::gazebo::msgs::RestResponse_Type RestResponse::type() const {
  return static_cast< ::gazebo::msgs::RestResponse_Type >(type_);
}
inline void RestResponse::set_type(::gazebo::msgs::RestResponse_Type value) {
  assert(::gazebo::msgs::RestResponse_Type_IsValid(value));
  set_has_type();
  type_ = value;
}

// optional string msg = 3;
inline bool RestResponse::has_msg() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void RestResponse::set_has_msg() {
  _has_bits_[0] |= 0x00000004u;
}
inline void RestResponse::clear_has_msg() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void RestResponse::clear_msg() {
  if (msg_ != &::google::protobuf::internal::kEmptyString) {
    msg_->clear();
  }
  clear_has_msg();
}
inline const ::std::string& RestResponse::msg() const {
  return *msg_;
}
inline void RestResponse::set_msg(const ::std::string& value) {
  set_has_msg();
  if (msg_ == &::google::protobuf::internal::kEmptyString) {
    msg_ = new ::std::string;
  }
  msg_->assign(value);
}
inline void RestResponse::set_msg(const char* value) {
  set_has_msg();
  if (msg_ == &::google::protobuf::internal::kEmptyString) {
    msg_ = new ::std::string;
  }
  msg_->assign(value);
}
inline void RestResponse::set_msg(const char* value, size_t size) {
  set_has_msg();
  if (msg_ == &::google::protobuf::internal::kEmptyString) {
    msg_ = new ::std::string;
  }
  msg_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* RestResponse::mutable_msg() {
  set_has_msg();
  if (msg_ == &::google::protobuf::internal::kEmptyString) {
    msg_ = new ::std::string;
  }
  return msg_;
}
inline ::std::string* RestResponse::release_msg() {
  clear_has_msg();
  if (msg_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = msg_;
    msg_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void RestResponse::set_allocated_msg(::std::string* msg) {
  if (msg_ != &::google::protobuf::internal::kEmptyString) {
    delete msg_;
  }
  if (msg) {
    set_has_msg();
    msg_ = msg;
  } else {
    clear_has_msg();
    msg_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}


typedef boost::shared_ptr<gazebo::msgs::RestResponse> RestResponsePtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gazebo::msgs::RestResponse_Type>() {
  return ::gazebo::msgs::RestResponse_Type_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::RestResponse const> ConstRestResponsePtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_rest_5fresponse_2eproto__INCLUDED
