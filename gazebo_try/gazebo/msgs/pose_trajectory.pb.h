// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pose_trajectory.proto

#ifndef PROTOBUF_pose_5ftrajectory_2eproto__INCLUDED
#define PROTOBUF_pose_5ftrajectory_2eproto__INCLUDED

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
#include "pose_stamped.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_pose_5ftrajectory_2eproto();
void protobuf_AssignDesc_pose_5ftrajectory_2eproto();
void protobuf_ShutdownFile_pose_5ftrajectory_2eproto();

class PoseTrajectory;

// ===================================================================

class GZ_MSGS_VISIBLE PoseTrajectory : public ::google::protobuf::Message {
 public:
  PoseTrajectory();
  virtual ~PoseTrajectory();

  PoseTrajectory(const PoseTrajectory& from);

  inline PoseTrajectory& operator=(const PoseTrajectory& from) {
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
  static const PoseTrajectory& default_instance();

  void Swap(PoseTrajectory* other);

  // implements Message ----------------------------------------------

  PoseTrajectory* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const PoseTrajectory& from);
  void MergeFrom(const PoseTrajectory& from);
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

  // optional string name = 1;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 1;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  inline void set_allocated_name(::std::string* name);

  // optional uint32 id = 2;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 2;
  inline ::google::protobuf::uint32 id() const;
  inline void set_id(::google::protobuf::uint32 value);

  // repeated .gazebo.msgs.PoseStamped pose_stamped = 3;
  inline int pose_stamped_size() const;
  inline void clear_pose_stamped();
  static const int kPoseStampedFieldNumber = 3;
  inline const ::gazebo::msgs::PoseStamped& pose_stamped(int index) const;
  inline ::gazebo::msgs::PoseStamped* mutable_pose_stamped(int index);
  inline ::gazebo::msgs::PoseStamped* add_pose_stamped();
  inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::PoseStamped >&
      pose_stamped() const;
  inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::PoseStamped >*
      mutable_pose_stamped();

  // @@protoc_insertion_point(class_scope:gazebo.msgs.PoseTrajectory)
 private:
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_id();
  inline void clear_has_id();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* name_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::PoseStamped > pose_stamped_;
  ::google::protobuf::uint32 id_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_pose_5ftrajectory_2eproto();
  friend void protobuf_AssignDesc_pose_5ftrajectory_2eproto();
  friend void protobuf_ShutdownFile_pose_5ftrajectory_2eproto();

  void InitAsDefaultInstance();
  static PoseTrajectory* default_instance_;
};
// ===================================================================


// ===================================================================

// PoseTrajectory

// optional string name = 1;
inline bool PoseTrajectory::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PoseTrajectory::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PoseTrajectory::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PoseTrajectory::clear_name() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& PoseTrajectory::name() const {
  return *name_;
}
inline void PoseTrajectory::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void PoseTrajectory::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void PoseTrajectory::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* PoseTrajectory::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  return name_;
}
inline ::std::string* PoseTrajectory::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void PoseTrajectory::set_allocated_name(::std::string* name) {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    delete name_;
  }
  if (name) {
    set_has_name();
    name_ = name;
  } else {
    clear_has_name();
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional uint32 id = 2;
inline bool PoseTrajectory::has_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PoseTrajectory::set_has_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PoseTrajectory::clear_has_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void PoseTrajectory::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 PoseTrajectory::id() const {
  return id_;
}
inline void PoseTrajectory::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
}

// repeated .gazebo.msgs.PoseStamped pose_stamped = 3;
inline int PoseTrajectory::pose_stamped_size() const {
  return pose_stamped_.size();
}
inline void PoseTrajectory::clear_pose_stamped() {
  pose_stamped_.Clear();
}
inline const ::gazebo::msgs::PoseStamped& PoseTrajectory::pose_stamped(int index) const {
  return pose_stamped_.Get(index);
}
inline ::gazebo::msgs::PoseStamped* PoseTrajectory::mutable_pose_stamped(int index) {
  return pose_stamped_.Mutable(index);
}
inline ::gazebo::msgs::PoseStamped* PoseTrajectory::add_pose_stamped() {
  return pose_stamped_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::PoseStamped >&
PoseTrajectory::pose_stamped() const {
  return pose_stamped_;
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::PoseStamped >*
PoseTrajectory::mutable_pose_stamped() {
  return &pose_stamped_;
}


typedef boost::shared_ptr<gazebo::msgs::PoseTrajectory> PoseTrajectoryPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::PoseTrajectory const> ConstPoseTrajectoryPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_pose_5ftrajectory_2eproto__INCLUDED
