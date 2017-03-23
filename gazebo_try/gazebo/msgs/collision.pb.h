// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: collision.proto

#ifndef PROTOBUF_collision_2eproto__INCLUDED
#define PROTOBUF_collision_2eproto__INCLUDED

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
#include "geometry.pb.h"
#include "surface.pb.h"
#include "visual.pb.h"
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_collision_2eproto();
void protobuf_AssignDesc_collision_2eproto();
void protobuf_ShutdownFile_collision_2eproto();

class Collision;

// ===================================================================

class GZ_MSGS_VISIBLE Collision : public ::google::protobuf::Message {
 public:
  Collision();
  virtual ~Collision();

  Collision(const Collision& from);

  inline Collision& operator=(const Collision& from) {
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
  static const Collision& default_instance();

  void Swap(Collision* other);

  // implements Message ----------------------------------------------

  Collision* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Collision& from);
  void MergeFrom(const Collision& from);
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

  // required uint32 id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline ::google::protobuf::uint32 id() const;
  inline void set_id(::google::protobuf::uint32 value);

  // required string name = 2;
  inline bool has_name() const;
  inline void clear_name();
  static const int kNameFieldNumber = 2;
  inline const ::std::string& name() const;
  inline void set_name(const ::std::string& value);
  inline void set_name(const char* value);
  inline void set_name(const char* value, size_t size);
  inline ::std::string* mutable_name();
  inline ::std::string* release_name();
  inline void set_allocated_name(::std::string* name);

  // optional double laser_retro = 3;
  inline bool has_laser_retro() const;
  inline void clear_laser_retro();
  static const int kLaserRetroFieldNumber = 3;
  inline double laser_retro() const;
  inline void set_laser_retro(double value);

  // optional double max_contacts = 4;
  inline bool has_max_contacts() const;
  inline void clear_max_contacts();
  static const int kMaxContactsFieldNumber = 4;
  inline double max_contacts() const;
  inline void set_max_contacts(double value);

  // optional .gazebo.msgs.Pose pose = 5;
  inline bool has_pose() const;
  inline void clear_pose();
  static const int kPoseFieldNumber = 5;
  inline const ::gazebo::msgs::Pose& pose() const;
  inline ::gazebo::msgs::Pose* mutable_pose();
  inline ::gazebo::msgs::Pose* release_pose();
  inline void set_allocated_pose(::gazebo::msgs::Pose* pose);

  // optional .gazebo.msgs.Geometry geometry = 6;
  inline bool has_geometry() const;
  inline void clear_geometry();
  static const int kGeometryFieldNumber = 6;
  inline const ::gazebo::msgs::Geometry& geometry() const;
  inline ::gazebo::msgs::Geometry* mutable_geometry();
  inline ::gazebo::msgs::Geometry* release_geometry();
  inline void set_allocated_geometry(::gazebo::msgs::Geometry* geometry);

  // optional .gazebo.msgs.Surface surface = 7;
  inline bool has_surface() const;
  inline void clear_surface();
  static const int kSurfaceFieldNumber = 7;
  inline const ::gazebo::msgs::Surface& surface() const;
  inline ::gazebo::msgs::Surface* mutable_surface();
  inline ::gazebo::msgs::Surface* release_surface();
  inline void set_allocated_surface(::gazebo::msgs::Surface* surface);

  // repeated .gazebo.msgs.Visual visual = 8;
  inline int visual_size() const;
  inline void clear_visual();
  static const int kVisualFieldNumber = 8;
  inline const ::gazebo::msgs::Visual& visual(int index) const;
  inline ::gazebo::msgs::Visual* mutable_visual(int index);
  inline ::gazebo::msgs::Visual* add_visual();
  inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Visual >&
      visual() const;
  inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Visual >*
      mutable_visual();

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Collision)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_name();
  inline void clear_has_name();
  inline void set_has_laser_retro();
  inline void clear_has_laser_retro();
  inline void set_has_max_contacts();
  inline void clear_has_max_contacts();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_geometry();
  inline void clear_has_geometry();
  inline void set_has_surface();
  inline void clear_has_surface();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* name_;
  double laser_retro_;
  double max_contacts_;
  ::gazebo::msgs::Pose* pose_;
  ::gazebo::msgs::Geometry* geometry_;
  ::gazebo::msgs::Surface* surface_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Visual > visual_;
  ::google::protobuf::uint32 id_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(8 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_collision_2eproto();
  friend void protobuf_AssignDesc_collision_2eproto();
  friend void protobuf_ShutdownFile_collision_2eproto();

  void InitAsDefaultInstance();
  static Collision* default_instance_;
};
// ===================================================================


// ===================================================================

// Collision

// required uint32 id = 1;
inline bool Collision::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Collision::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Collision::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Collision::clear_id() {
  id_ = 0u;
  clear_has_id();
}
inline ::google::protobuf::uint32 Collision::id() const {
  return id_;
}
inline void Collision::set_id(::google::protobuf::uint32 value) {
  set_has_id();
  id_ = value;
}

// required string name = 2;
inline bool Collision::has_name() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Collision::set_has_name() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Collision::clear_has_name() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Collision::clear_name() {
  if (name_ != &::google::protobuf::internal::kEmptyString) {
    name_->clear();
  }
  clear_has_name();
}
inline const ::std::string& Collision::name() const {
  return *name_;
}
inline void Collision::set_name(const ::std::string& value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Collision::set_name(const char* value) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(value);
}
inline void Collision::set_name(const char* value, size_t size) {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  name_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Collision::mutable_name() {
  set_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    name_ = new ::std::string;
  }
  return name_;
}
inline ::std::string* Collision::release_name() {
  clear_has_name();
  if (name_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = name_;
    name_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Collision::set_allocated_name(::std::string* name) {
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

// optional double laser_retro = 3;
inline bool Collision::has_laser_retro() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Collision::set_has_laser_retro() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Collision::clear_has_laser_retro() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Collision::clear_laser_retro() {
  laser_retro_ = 0;
  clear_has_laser_retro();
}
inline double Collision::laser_retro() const {
  return laser_retro_;
}
inline void Collision::set_laser_retro(double value) {
  set_has_laser_retro();
  laser_retro_ = value;
}

// optional double max_contacts = 4;
inline bool Collision::has_max_contacts() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Collision::set_has_max_contacts() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Collision::clear_has_max_contacts() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Collision::clear_max_contacts() {
  max_contacts_ = 0;
  clear_has_max_contacts();
}
inline double Collision::max_contacts() const {
  return max_contacts_;
}
inline void Collision::set_max_contacts(double value) {
  set_has_max_contacts();
  max_contacts_ = value;
}

// optional .gazebo.msgs.Pose pose = 5;
inline bool Collision::has_pose() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Collision::set_has_pose() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Collision::clear_has_pose() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Collision::clear_pose() {
  if (pose_ != NULL) pose_->::gazebo::msgs::Pose::Clear();
  clear_has_pose();
}
inline const ::gazebo::msgs::Pose& Collision::pose() const {
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::gazebo::msgs::Pose* Collision::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) pose_ = new ::gazebo::msgs::Pose;
  return pose_;
}
inline ::gazebo::msgs::Pose* Collision::release_pose() {
  clear_has_pose();
  ::gazebo::msgs::Pose* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void Collision::set_allocated_pose(::gazebo::msgs::Pose* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
}

// optional .gazebo.msgs.Geometry geometry = 6;
inline bool Collision::has_geometry() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Collision::set_has_geometry() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Collision::clear_has_geometry() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Collision::clear_geometry() {
  if (geometry_ != NULL) geometry_->::gazebo::msgs::Geometry::Clear();
  clear_has_geometry();
}
inline const ::gazebo::msgs::Geometry& Collision::geometry() const {
  return geometry_ != NULL ? *geometry_ : *default_instance_->geometry_;
}
inline ::gazebo::msgs::Geometry* Collision::mutable_geometry() {
  set_has_geometry();
  if (geometry_ == NULL) geometry_ = new ::gazebo::msgs::Geometry;
  return geometry_;
}
inline ::gazebo::msgs::Geometry* Collision::release_geometry() {
  clear_has_geometry();
  ::gazebo::msgs::Geometry* temp = geometry_;
  geometry_ = NULL;
  return temp;
}
inline void Collision::set_allocated_geometry(::gazebo::msgs::Geometry* geometry) {
  delete geometry_;
  geometry_ = geometry;
  if (geometry) {
    set_has_geometry();
  } else {
    clear_has_geometry();
  }
}

// optional .gazebo.msgs.Surface surface = 7;
inline bool Collision::has_surface() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Collision::set_has_surface() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Collision::clear_has_surface() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Collision::clear_surface() {
  if (surface_ != NULL) surface_->::gazebo::msgs::Surface::Clear();
  clear_has_surface();
}
inline const ::gazebo::msgs::Surface& Collision::surface() const {
  return surface_ != NULL ? *surface_ : *default_instance_->surface_;
}
inline ::gazebo::msgs::Surface* Collision::mutable_surface() {
  set_has_surface();
  if (surface_ == NULL) surface_ = new ::gazebo::msgs::Surface;
  return surface_;
}
inline ::gazebo::msgs::Surface* Collision::release_surface() {
  clear_has_surface();
  ::gazebo::msgs::Surface* temp = surface_;
  surface_ = NULL;
  return temp;
}
inline void Collision::set_allocated_surface(::gazebo::msgs::Surface* surface) {
  delete surface_;
  surface_ = surface;
  if (surface) {
    set_has_surface();
  } else {
    clear_has_surface();
  }
}

// repeated .gazebo.msgs.Visual visual = 8;
inline int Collision::visual_size() const {
  return visual_.size();
}
inline void Collision::clear_visual() {
  visual_.Clear();
}
inline const ::gazebo::msgs::Visual& Collision::visual(int index) const {
  return visual_.Get(index);
}
inline ::gazebo::msgs::Visual* Collision::mutable_visual(int index) {
  return visual_.Mutable(index);
}
inline ::gazebo::msgs::Visual* Collision::add_visual() {
  return visual_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Visual >&
Collision::visual() const {
  return visual_;
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Visual >*
Collision::mutable_visual() {
  return &visual_;
}


typedef boost::shared_ptr<gazebo::msgs::Collision> CollisionPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Collision const> ConstCollisionPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_collision_2eproto__INCLUDED
