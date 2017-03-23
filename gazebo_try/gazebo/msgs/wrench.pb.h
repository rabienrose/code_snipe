// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: wrench.proto

#ifndef PROTOBUF_wrench_2eproto__INCLUDED
#define PROTOBUF_wrench_2eproto__INCLUDED

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
#include "vector3d.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_wrench_2eproto();
void protobuf_AssignDesc_wrench_2eproto();
void protobuf_ShutdownFile_wrench_2eproto();

class Wrench;

// ===================================================================

class GZ_MSGS_VISIBLE Wrench : public ::google::protobuf::Message {
 public:
  Wrench();
  virtual ~Wrench();

  Wrench(const Wrench& from);

  inline Wrench& operator=(const Wrench& from) {
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
  static const Wrench& default_instance();

  void Swap(Wrench* other);

  // implements Message ----------------------------------------------

  Wrench* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Wrench& from);
  void MergeFrom(const Wrench& from);
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

  // required .gazebo.msgs.Vector3d force = 1;
  inline bool has_force() const;
  inline void clear_force();
  static const int kForceFieldNumber = 1;
  inline const ::gazebo::msgs::Vector3d& force() const;
  inline ::gazebo::msgs::Vector3d* mutable_force();
  inline ::gazebo::msgs::Vector3d* release_force();
  inline void set_allocated_force(::gazebo::msgs::Vector3d* force);

  // required .gazebo.msgs.Vector3d torque = 2;
  inline bool has_torque() const;
  inline void clear_torque();
  static const int kTorqueFieldNumber = 2;
  inline const ::gazebo::msgs::Vector3d& torque() const;
  inline ::gazebo::msgs::Vector3d* mutable_torque();
  inline ::gazebo::msgs::Vector3d* release_torque();
  inline void set_allocated_torque(::gazebo::msgs::Vector3d* torque);

  // optional .gazebo.msgs.Vector3d force_offset = 3;
  inline bool has_force_offset() const;
  inline void clear_force_offset();
  static const int kForceOffsetFieldNumber = 3;
  inline const ::gazebo::msgs::Vector3d& force_offset() const;
  inline ::gazebo::msgs::Vector3d* mutable_force_offset();
  inline ::gazebo::msgs::Vector3d* release_force_offset();
  inline void set_allocated_force_offset(::gazebo::msgs::Vector3d* force_offset);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Wrench)
 private:
  inline void set_has_force();
  inline void clear_has_force();
  inline void set_has_torque();
  inline void clear_has_torque();
  inline void set_has_force_offset();
  inline void clear_has_force_offset();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::gazebo::msgs::Vector3d* force_;
  ::gazebo::msgs::Vector3d* torque_;
  ::gazebo::msgs::Vector3d* force_offset_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_wrench_2eproto();
  friend void protobuf_AssignDesc_wrench_2eproto();
  friend void protobuf_ShutdownFile_wrench_2eproto();

  void InitAsDefaultInstance();
  static Wrench* default_instance_;
};
// ===================================================================


// ===================================================================

// Wrench

// required .gazebo.msgs.Vector3d force = 1;
inline bool Wrench::has_force() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Wrench::set_has_force() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Wrench::clear_has_force() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Wrench::clear_force() {
  if (force_ != NULL) force_->::gazebo::msgs::Vector3d::Clear();
  clear_has_force();
}
inline const ::gazebo::msgs::Vector3d& Wrench::force() const {
  return force_ != NULL ? *force_ : *default_instance_->force_;
}
inline ::gazebo::msgs::Vector3d* Wrench::mutable_force() {
  set_has_force();
  if (force_ == NULL) force_ = new ::gazebo::msgs::Vector3d;
  return force_;
}
inline ::gazebo::msgs::Vector3d* Wrench::release_force() {
  clear_has_force();
  ::gazebo::msgs::Vector3d* temp = force_;
  force_ = NULL;
  return temp;
}
inline void Wrench::set_allocated_force(::gazebo::msgs::Vector3d* force) {
  delete force_;
  force_ = force;
  if (force) {
    set_has_force();
  } else {
    clear_has_force();
  }
}

// required .gazebo.msgs.Vector3d torque = 2;
inline bool Wrench::has_torque() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Wrench::set_has_torque() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Wrench::clear_has_torque() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Wrench::clear_torque() {
  if (torque_ != NULL) torque_->::gazebo::msgs::Vector3d::Clear();
  clear_has_torque();
}
inline const ::gazebo::msgs::Vector3d& Wrench::torque() const {
  return torque_ != NULL ? *torque_ : *default_instance_->torque_;
}
inline ::gazebo::msgs::Vector3d* Wrench::mutable_torque() {
  set_has_torque();
  if (torque_ == NULL) torque_ = new ::gazebo::msgs::Vector3d;
  return torque_;
}
inline ::gazebo::msgs::Vector3d* Wrench::release_torque() {
  clear_has_torque();
  ::gazebo::msgs::Vector3d* temp = torque_;
  torque_ = NULL;
  return temp;
}
inline void Wrench::set_allocated_torque(::gazebo::msgs::Vector3d* torque) {
  delete torque_;
  torque_ = torque;
  if (torque) {
    set_has_torque();
  } else {
    clear_has_torque();
  }
}

// optional .gazebo.msgs.Vector3d force_offset = 3;
inline bool Wrench::has_force_offset() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Wrench::set_has_force_offset() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Wrench::clear_has_force_offset() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Wrench::clear_force_offset() {
  if (force_offset_ != NULL) force_offset_->::gazebo::msgs::Vector3d::Clear();
  clear_has_force_offset();
}
inline const ::gazebo::msgs::Vector3d& Wrench::force_offset() const {
  return force_offset_ != NULL ? *force_offset_ : *default_instance_->force_offset_;
}
inline ::gazebo::msgs::Vector3d* Wrench::mutable_force_offset() {
  set_has_force_offset();
  if (force_offset_ == NULL) force_offset_ = new ::gazebo::msgs::Vector3d;
  return force_offset_;
}
inline ::gazebo::msgs::Vector3d* Wrench::release_force_offset() {
  clear_has_force_offset();
  ::gazebo::msgs::Vector3d* temp = force_offset_;
  force_offset_ = NULL;
  return temp;
}
inline void Wrench::set_allocated_force_offset(::gazebo::msgs::Vector3d* force_offset) {
  delete force_offset_;
  force_offset_ = force_offset;
  if (force_offset) {
    set_has_force_offset();
  } else {
    clear_has_force_offset();
  }
}


typedef boost::shared_ptr<gazebo::msgs::Wrench> WrenchPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Wrench const> ConstWrenchPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_wrench_2eproto__INCLUDED
