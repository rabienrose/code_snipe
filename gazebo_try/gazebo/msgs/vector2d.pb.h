// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vector2d.proto

#ifndef PROTOBUF_vector2d_2eproto__INCLUDED
#define PROTOBUF_vector2d_2eproto__INCLUDED

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
#pragma GCC system_header
#include <boost/shared_ptr.hpp>
#include <gazebo/util/system.hh>
#include "gazebo/msgs/MsgFactory.hh"
// @@protoc_insertion_point(includes)

namespace gazebo {
namespace msgs {

// Internal implementation detail -- do not call these.
void GZ_MSGS_VISIBLE protobuf_AddDesc_vector2d_2eproto();
void protobuf_AssignDesc_vector2d_2eproto();
void protobuf_ShutdownFile_vector2d_2eproto();

class Vector2d;

// ===================================================================

class GZ_MSGS_VISIBLE Vector2d : public ::google::protobuf::Message {
 public:
  Vector2d();
  virtual ~Vector2d();

  Vector2d(const Vector2d& from);

  inline Vector2d& operator=(const Vector2d& from) {
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
  static const Vector2d& default_instance();

  void Swap(Vector2d* other);

  // implements Message ----------------------------------------------

  Vector2d* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Vector2d& from);
  void MergeFrom(const Vector2d& from);
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

  // required double x = 1;
  inline bool has_x() const;
  inline void clear_x();
  static const int kXFieldNumber = 1;
  inline double x() const;
  inline void set_x(double value);

  // required double y = 2;
  inline bool has_y() const;
  inline void clear_y();
  static const int kYFieldNumber = 2;
  inline double y() const;
  inline void set_y(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Vector2d)
 private:
  inline void set_has_x();
  inline void clear_has_x();
  inline void set_has_y();
  inline void clear_has_y();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  double x_;
  double y_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void GZ_MSGS_VISIBLE protobuf_AddDesc_vector2d_2eproto();
  friend void protobuf_AssignDesc_vector2d_2eproto();
  friend void protobuf_ShutdownFile_vector2d_2eproto();

  void InitAsDefaultInstance();
  static Vector2d* default_instance_;
};
// ===================================================================


// ===================================================================

// Vector2d

// required double x = 1;
inline bool Vector2d::has_x() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Vector2d::set_has_x() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Vector2d::clear_has_x() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Vector2d::clear_x() {
  x_ = 0;
  clear_has_x();
}
inline double Vector2d::x() const {
  return x_;
}
inline void Vector2d::set_x(double value) {
  set_has_x();
  x_ = value;
}

// required double y = 2;
inline bool Vector2d::has_y() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Vector2d::set_has_y() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Vector2d::clear_has_y() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Vector2d::clear_y() {
  y_ = 0;
  clear_has_y();
}
inline double Vector2d::y() const {
  return y_;
}
inline void Vector2d::set_y(double value) {
  set_has_y();
  y_ = value;
}


typedef boost::shared_ptr<gazebo::msgs::Vector2d> Vector2dPtr;
// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

typedef const boost::shared_ptr<gazebo::msgs::Vector2d const> ConstVector2dPtr;
// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_vector2d_2eproto__INCLUDED
