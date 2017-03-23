// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: factory.proto

#ifndef PROTOBUF_factory_2eproto__INCLUDED
#define PROTOBUF_factory_2eproto__INCLUDED

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
#include "pose.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_factory_2eproto();
void protobuf_AssignDesc_factory_2eproto();
void protobuf_ShutdownFile_factory_2eproto();

class Factory;

// ===================================================================

class GZ_MSGS_VISIBLE Factory : public ::google::protobuf::Message {
 public:
  Factory();
  virtual ~Factory();

  Factory(const Factory& from);

  inline Factory& operator=(const Factory& from) {
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
  static const Factory& default_instance();

  void Swap(Factory* other);

  // implements Message ----------------------------------------------

  Factory* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Factory& from);
  void MergeFrom(const Factory& from);
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

  // optional string sdf = 1;
  inline bool has_sdf() const;
  inline void clear_sdf();
  static const int kSdfFieldNumber = 1;
  inline const ::std::string& sdf() const;
  inline void set_sdf(const ::std::string& value);
  inline void set_sdf(const char* value);
  inline void set_sdf(const char* value, size_t size);
  inline ::std::string* mutable_sdf();
  inline ::std::string* release_sdf();
  inline void set_allocated_sdf(::std::string* sdf);

  // optional string sdf_filename = 2;
  inline bool has_sdf_filename() const;
  inline void clear_sdf_filename();
  static const int kSdfFilenameFieldNumber = 2;
  inline const ::std::string& sdf_filename() const;
  inline void set_sdf_filename(const ::std::string& value);
  inline void set_sdf_filename(const char* value);
  inline void set_sdf_filename(const char* value, size_t size);
  inline ::std::string* mutable_sdf_filename();
  inline ::std::string* release_sdf_filename();
  inline void set_allocated_sdf_filename(::std::string* sdf_filename);

  // optional .gazebo.msgs.Pose pose = 3;
  inline bool has_pose() const;
  inline void clear_pose();
  static const int kPoseFieldNumber = 3;
  inline const ::gazebo::msgs::Pose& pose() const;
  inline ::gazebo::msgs::Pose* mutable_pose();
  inline ::gazebo::msgs::Pose* release_pose();
  inline void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // optional string edit_name = 4;
  inline bool has_edit_name() const;
  inline void clear_edit_name();
  static const int kEditNameFieldNumber = 4;
  inline const ::std::string& edit_name() const;
  inline void set_edit_name(const ::std::string& value);
  inline void set_edit_name(const char* value);
  inline void set_edit_name(const char* value, size_t size);
  inline ::std::string* mutable_edit_name();
  inline ::std::string* release_edit_name();
  inline void set_allocated_edit_name(::std::string* edit_name);

  // optional string clone_model_name = 5;
  inline bool has_clone_model_name() const;
  inline void clear_clone_model_name();
  static const int kCloneModelNameFieldNumber = 5;
  inline const ::std::string& clone_model_name() const;
  inline void set_clone_model_name(const ::std::string& value);
  inline void set_clone_model_name(const char* value);
  inline void set_clone_model_name(const char* value, size_t size);
  inline ::std::string* mutable_clone_model_name();
  inline ::std::string* release_clone_model_name();
  inline void set_allocated_clone_model_name(::std::string* clone_model_name);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Factory)
 private:
  inline void set_has_sdf();
  inline void clear_has_sdf();
  inline void set_has_sdf_filename();
  inline void clear_has_sdf_filename();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_edit_name();
  inline void clear_has_edit_name();
  inline void set_has_clone_model_name();
  inline void clear_has_clone_model_name();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* sdf_;
  ::std::string* sdf_filename_;
  ::gazebo::msgs::Pose* pose_;
  ::std::string* edit_name_;
  ::std::string* clone_model_name_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(5 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_factory_2eproto();
  friend void protobuf_AssignDesc_factory_2eproto();
  friend void protobuf_ShutdownFile_factory_2eproto();

  void InitAsDefaultInstance();
  static Factory* default_instance_;
};
// ===================================================================


// ===================================================================

// Factory

// optional string sdf = 1;
inline bool Factory::has_sdf() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Factory::set_has_sdf() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Factory::clear_has_sdf() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Factory::clear_sdf() {
  if (sdf_ != &::google::protobuf::internal::kEmptyString) {
    sdf_->clear();
  }
  clear_has_sdf();
}
inline const ::std::string& Factory::sdf() const {
  return *sdf_;
}
inline void Factory::set_sdf(const ::std::string& value) {
  set_has_sdf();
  if (sdf_ == &::google::protobuf::internal::kEmptyString) {
    sdf_ = new ::std::string;
  }
  sdf_->assign(value);
}
inline void Factory::set_sdf(const char* value) {
  set_has_sdf();
  if (sdf_ == &::google::protobuf::internal::kEmptyString) {
    sdf_ = new ::std::string;
  }
  sdf_->assign(value);
}
inline void Factory::set_sdf(const char* value, size_t size) {
  set_has_sdf();
  if (sdf_ == &::google::protobuf::internal::kEmptyString) {
    sdf_ = new ::std::string;
  }
  sdf_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Factory::mutable_sdf() {
  set_has_sdf();
  if (sdf_ == &::google::protobuf::internal::kEmptyString) {
    sdf_ = new ::std::string;
  }
  return sdf_;
}
inline ::std::string* Factory::release_sdf() {
  clear_has_sdf();
  if (sdf_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = sdf_;
    sdf_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Factory::set_allocated_sdf(::std::string* sdf) {
  if (sdf_ != &::google::protobuf::internal::kEmptyString) {
    delete sdf_;
  }
  if (sdf) {
    set_has_sdf();
    sdf_ = sdf;
  } else {
    clear_has_sdf();
    sdf_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional string sdf_filename = 2;
inline bool Factory::has_sdf_filename() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Factory::set_has_sdf_filename() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Factory::clear_has_sdf_filename() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Factory::clear_sdf_filename() {
  if (sdf_filename_ != &::google::protobuf::internal::kEmptyString) {
    sdf_filename_->clear();
  }
  clear_has_sdf_filename();
}
inline const ::std::string& Factory::sdf_filename() const {
  return *sdf_filename_;
}
inline void Factory::set_sdf_filename(const ::std::string& value) {
  set_has_sdf_filename();
  if (sdf_filename_ == &::google::protobuf::internal::kEmptyString) {
    sdf_filename_ = new ::std::string;
  }
  sdf_filename_->assign(value);
}
inline void Factory::set_sdf_filename(const char* value) {
  set_has_sdf_filename();
  if (sdf_filename_ == &::google::protobuf::internal::kEmptyString) {
    sdf_filename_ = new ::std::string;
  }
  sdf_filename_->assign(value);
}
inline void Factory::set_sdf_filename(const char* value, size_t size) {
  set_has_sdf_filename();
  if (sdf_filename_ == &::google::protobuf::internal::kEmptyString) {
    sdf_filename_ = new ::std::string;
  }
  sdf_filename_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Factory::mutable_sdf_filename() {
  set_has_sdf_filename();
  if (sdf_filename_ == &::google::protobuf::internal::kEmptyString) {
    sdf_filename_ = new ::std::string;
  }
  return sdf_filename_;
}
inline ::std::string* Factory::release_sdf_filename() {
  clear_has_sdf_filename();
  if (sdf_filename_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = sdf_filename_;
    sdf_filename_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Factory::set_allocated_sdf_filename(::std::string* sdf_filename) {
  if (sdf_filename_ != &::google::protobuf::internal::kEmptyString) {
    delete sdf_filename_;
  }
  if (sdf_filename) {
    set_has_sdf_filename();
    sdf_filename_ = sdf_filename;
  } else {
    clear_has_sdf_filename();
    sdf_filename_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional .gazebo.msgs.Pose pose = 3;
inline bool Factory::has_pose() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Factory::set_has_pose() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Factory::clear_has_pose() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Factory::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& Factory::pose() const {
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* Factory::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) pose_ = new ::gazebo::msgs::Pose;
  return pose_;
}
inline ::gazebo::msgs::Pose* Factory::release_pose() {
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void Factory::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
}

// optional string edit_name = 4;
inline bool Factory::has_edit_name() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Factory::set_has_edit_name() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Factory::clear_has_edit_name() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Factory::clear_edit_name() {
  if (edit_name_ != &::google::protobuf::internal::kEmptyString) {
    edit_name_->clear();
  }
  clear_has_edit_name();
}
inline const ::std::string& Factory::edit_name() const {
  return *edit_name_;
}
inline void Factory::set_edit_name(const ::std::string& value) {
  set_has_edit_name();
  if (edit_name_ == &::google::protobuf::internal::kEmptyString) {
    edit_name_ = new ::std::string;
  }
  edit_name_->assign(value);
}
inline void Factory::set_edit_name(const char* value) {
  set_has_edit_name();
  if (edit_name_ == &::google::protobuf::internal::kEmptyString) {
    edit_name_ = new ::std::string;
  }
  edit_name_->assign(value);
}
inline void Factory::set_edit_name(const char* value, size_t size) {
  set_has_edit_name();
  if (edit_name_ == &::google::protobuf::internal::kEmptyString) {
    edit_name_ = new ::std::string;
  }
  edit_name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Factory::mutable_edit_name() {
  set_has_edit_name();
  if (edit_name_ == &::google::protobuf::internal::kEmptyString) {
    edit_name_ = new ::std::string;
  }
  return edit_name_;
}
inline ::std::string* Factory::release_edit_name() {
  clear_has_edit_name();
  if (edit_name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = edit_name_;
    edit_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Factory::set_allocated_edit_name(::std::string* edit_name) {
  if (edit_name_ != &::google::protobuf::internal::kEmptyString) {
    delete edit_name_;
  }
  if (edit_name) {
    set_has_edit_name();
    edit_name_ = edit_name;
  } else {
    clear_has_edit_name();
    edit_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional string clone_model_name = 5;
inline bool Factory::has_clone_model_name() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Factory::set_has_clone_model_name() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Factory::clear_has_clone_model_name() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Factory::clear_clone_model_name() {
  if (clone_model_name_ != &::google::protobuf::internal::kEmptyString) {
    clone_model_name_->clear();
  }
  clear_has_clone_model_name();
}
inline const ::std::string& Factory::clone_model_name() const {
  return *clone_model_name_;
}
inline void Factory::set_clone_model_name(const ::std::string& value) {
  set_has_clone_model_name();
  if (clone_model_name_ == &::google::protobuf::internal::kEmptyString) {
    clone_model_name_ = new ::std::string;
  }
  clone_model_name_->assign(value);
}
inline void Factory::set_clone_model_name(const char* value) {
  set_has_clone_model_name();
  if (clone_model_name_ == &::google::protobuf::internal::kEmptyString) {
    clone_model_name_ = new ::std::string;
  }
  clone_model_name_->assign(value);
}
inline void Factory::set_clone_model_name(const char* value, size_t size) {
  set_has_clone_model_name();
  if (clone_model_name_ == &::google::protobuf::internal::kEmptyString) {
    clone_model_name_ = new ::std::string;
  }
  clone_model_name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Factory::mutable_clone_model_name() {
  set_has_clone_model_name();
  if (clone_model_name_ == &::google::protobuf::internal::kEmptyString) {
    clone_model_name_ = new ::std::string;
  }
  return clone_model_name_;
}
inline ::std::string* Factory::release_clone_model_name() {
  clear_has_clone_model_name();
  if (clone_model_name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = clone_model_name_;
    clone_model_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Factory::set_allocated_clone_model_name(::std::string* clone_model_name) {
  if (clone_model_name_ != &::google::protobuf::internal::kEmptyString) {
    delete clone_model_name_;
  }
  if (clone_model_name) {
    set_has_clone_model_name();
    clone_model_name_ = clone_model_name;
  } else {
    clear_has_clone_model_name();
    clone_model_name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}


typedef boost::shared_ptr<gazebo::msgs::Factory> FactoryPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Factory const> ConstFactoryPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_factory_2eproto__INCLUDED
