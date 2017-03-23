// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: wrench_stamped.proto

#ifndef PROTOBUF_wrench_5fstamped_2eproto__INCLUDED
#define PROTOBUF_wrench_5fstamped_2eproto__INCLUDED

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
#include "wrench.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_wrench_5fstamped_2eproto();
void protobuf_AssignDesc_wrench_5fstamped_2eproto();
void protobuf_ShutdownFile_wrench_5fstamped_2eproto();

class WrenchStamped;

// ===================================================================

class GZ_MSGS_VISIBLE WrenchStamped : public ::google::protobuf::Message {
 public:
  WrenchStamped();
  virtual ~WrenchStamped();

  WrenchStamped(const WrenchStamped& from);

  inline WrenchStamped& operator=(const WrenchStamped& from) {
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
  static const WrenchStamped& default_instance();

  void Swap(WrenchStamped* other);

  // implements Message ----------------------------------------------

  WrenchStamped* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const WrenchStamped& from);
  void MergeFrom(const WrenchStamped& from);
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

  // required .gazebo.msgs.Wrench wrench = 2;
  inline bool has_wrench() const;
  inline void clear_wrench();
  static const int kWrenchFieldNumber = 2;
  inline const ::gazebo::msgs::Wrench& wrench() const;
  inline ::gazebo::msgs::Wrench* mutable_wrench();
  inline ::gazebo::msgs::Wrench* release_wrench();
  inline void set_allocated_wrench(::gazebo::msgs::Wrench* wrench);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.WrenchStamped)
 private:
  inline void set_has_time();
  inline void clear_has_time();
  inline void set_has_wrench();
  inline void clear_has_wrench();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::gazebo::msgs::Time* time_;
  ::gazebo::msgs::Wrench* wrench_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_wrench_5fstamped_2eproto();
  friend void protobuf_AssignDesc_wrench_5fstamped_2eproto();
  friend void protobuf_ShutdownFile_wrench_5fstamped_2eproto();

  void InitAsDefaultInstance();
  static WrenchStamped* default_instance_;
};
// ===================================================================


// ===================================================================

// WrenchStamped

// required .gazebo.msgs.Time time = 1;
inline bool WrenchStamped::has_time() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void WrenchStamped::set_has_time() {
  _has_bits_[0] |= 0x00000001u;
}
inline void WrenchStamped::clear_has_time() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void WrenchStamped::clear_time() {
  if (time_ != NULL) time_->::gazebo::msgs::Time::Clear();
  clear_has_time();
}
inline const ::gazebo::msgs::Time& WrenchStamped::time() const {
  return time_ != NULL ? *time_ : *default_instance_->time_;
}
inline ::gazebo::msgs::Time* WrenchStamped::mutable_time() {
  set_has_time();
  if (time_ == NULL) time_ = new ::gazebo::msgs::Time;
  return time_;
}
inline ::gazebo::msgs::Time* WrenchStamped::release_time() {
  clear_has_time();
  ::gazebo::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline void WrenchStamped::set_allocated_time(::gazebo::msgs::Time* time) {
  delete time_;
  time_ = time;
  if (time) {
    set_has_time();
  } else {
    clear_has_time();
  }
}

// required .gazebo.msgs.Wrench wrench = 2;
inline bool WrenchStamped::has_wrench() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void WrenchStamped::set_has_wrench() {
  _has_bits_[0] |= 0x00000002u;
}
inline void WrenchStamped::clear_has_wrench() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void WrenchStamped::clear_wrench() {
  if (wrench_ != NULL) wrench_->::gazebo::msgs::Wrench::Clear();
  clear_has_wrench();
}
inline const ::gazebo::msgs::Wrench& WrenchStamped::wrench() const {
  return wrench_ != NULL ? *wrench_ : *default_instance_->wrench_;
}
inline ::gazebo::msgs::Wrench* WrenchStamped::mutable_wrench() {
  set_has_wrench();
  if (wrench_ == NULL) wrench_ = new ::gazebo::msgs::Wrench;
  return wrench_;
}
inline ::gazebo::msgs::Wrench* WrenchStamped::release_wrench() {
  clear_has_wrench();
  ::gazebo::msgs::Wrench* temp = wrench_;
  wrench_ = NULL;
  return temp;
}
inline void WrenchStamped::set_allocated_wrench(::gazebo::msgs::Wrench* wrench) {
  delete wrench_;
  wrench_ = wrench;
  if (wrench) {
    set_has_wrench();
  } else {
    clear_has_wrench();
  }
}


typedef boost::shared_ptr<gazebo::msgs::WrenchStamped> WrenchStampedPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::WrenchStamped const> ConstWrenchStampedPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_wrench_5fstamped_2eproto__INCLUDED
