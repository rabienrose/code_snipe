// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: magnetometer.proto

#ifndef PROTOBUF_magnetometer_2eproto__INCLUDED
#define PROTOBUF_magnetometer_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
#include "time.pb.h"
#include "vector3d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_magnetometer_2eproto();
void protobuf_AssignDesc_magnetometer_2eproto();
void protobuf_ShutdownFile_magnetometer_2eproto();

class Magnetometer;

// ===================================================================

class GZ_MSGS_VISIBLE Magnetometer : public ::google::protobuf::Message {
 public:
  Magnetometer();
  virtual ~Magnetometer();

  Magnetometer(const Magnetometer& from);

  inline Magnetometer& operator=(const Magnetometer& from) {
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
  static const Magnetometer& default_instance();

  void Swap(Magnetometer* other);

  // implements Message ----------------------------------------------

  Magnetometer* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Magnetometer& from);
  void MergeFrom(const Magnetometer& from);
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

  // accessors -------------------------------------------------------

  // required .gazebo.msgs.Time time = 1;
  inline bool has_time() const;
  inline void clear_time();
  static const int kTimeFieldNumber = 1;
  inline const ::gazebo::msgs::Time& time() const;
  inline ::gazebo::msgs::Time* mutable_time();
  inline ::gazebo::msgs::Time* release_time();
  inline void set_allocated_time(::gazebo::msgs::Time* time);

  // required .gazebo.msgs.Vector3d field_tesla = 2;
  inline bool has_field_tesla() const;
  inline void clear_field_tesla();
  static const int kFieldTeslaFieldNumber = 2;
  inline const ::gazebo::msgs::Vector3d& field_tesla() const;
  inline ::gazebo::msgs::Vector3d* mutable_field_tesla();
  inline ::gazebo::msgs::Vector3d* release_field_tesla();
  inline void set_allocated_field_tesla(::gazebo::msgs::Vector3d* field_tesla);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Magnetometer)
 private:
  inline void set_has_time();
  inline void clear_has_time();
  inline void set_has_field_tesla();
  inline void clear_has_field_tesla();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::gazebo::msgs::Time* time_;
  ::gazebo::msgs::Vector3d* field_tesla_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_magnetometer_2eproto();
  friend void protobuf_AssignDesc_magnetometer_2eproto();
  friend void protobuf_ShutdownFile_magnetometer_2eproto();

  void InitAsDefaultInstance();
  static Magnetometer* default_instance_;
};
// ===================================================================


// ===================================================================

// Magnetometer

// required .gazebo.msgs.Time time = 1;
inline bool Magnetometer::has_time() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Magnetometer::set_has_time() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Magnetometer::clear_has_time() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Magnetometer::clear_time() {
  if (time_ != NULL) time_->::gazebo::msgs::Time::Clear();
  clear_has_time();
}
inline const ::gazebo::msgs::Time& Magnetometer::time() const {
  return time_ != NULL ? *time_ : *default_instance_->time_;
}
inline ::gazebo::msgs::Time* Magnetometer::mutable_time() {
  set_has_time();
  if (time_ == NULL) time_ = new ::gazebo::msgs::Time;
  return time_;
}
inline ::gazebo::msgs::Time* Magnetometer::release_time() {
  clear_has_time();
  ::gazebo::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline void Magnetometer::set_allocated_time(::gazebo::msgs::Time* time) {
  delete time_;
  time_ = time;
  if (time) {
    set_has_time();
  } else {
    clear_has_time();
  }
}

// required .gazebo.msgs.Vector3d field_tesla = 2;
inline bool Magnetometer::has_field_tesla() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Magnetometer::set_has_field_tesla() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Magnetometer::clear_has_field_tesla() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Magnetometer::clear_field_tesla() {
  if (field_tesla_ != NULL) field_tesla_->::gazebo::msgs::Vector3d::Clear();
  clear_has_field_tesla();
}
inline const ::gazebo::msgs::Vector3d& Magnetometer::field_tesla() const {
  return field_tesla_ != NULL ? *field_tesla_ : *default_instance_->field_tesla_;
}
inline ::gazebo::msgs::Vector3d* Magnetometer::mutable_field_tesla() {
  set_has_field_tesla();
  if (field_tesla_ == NULL) field_tesla_ = new ::gazebo::msgs::Vector3d;
  return field_tesla_;
}
inline ::gazebo::msgs::Vector3d* Magnetometer::release_field_tesla() {
  clear_has_field_tesla();
  ::gazebo::msgs::Vector3d* temp = field_tesla_;
  field_tesla_ = NULL;
  return temp;
}
inline void Magnetometer::set_allocated_field_tesla(::gazebo::msgs::Vector3d* field_tesla) {
  delete field_tesla_;
  field_tesla_ = field_tesla;
  if (field_tesla) {
    set_has_field_tesla();
  } else {
    clear_has_field_tesla();
  }
}


typedef boost::shared_ptr<gazebo::msgs::Magnetometer> MagnetometerPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Magnetometer const> ConstMagnetometerPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_magnetometer_2eproto__INCLUDED
