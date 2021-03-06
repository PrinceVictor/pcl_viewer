// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: image_msg.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_image_5fmsg_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_image_5fmsg_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3012000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3012003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_image_5fmsg_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_image_5fmsg_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_image_5fmsg_2eproto;
namespace image_msg {
class image;
class imageDefaultTypeInternal;
extern imageDefaultTypeInternal _image_default_instance_;
class image_buf;
class image_bufDefaultTypeInternal;
extern image_bufDefaultTypeInternal _image_buf_default_instance_;
}  // namespace image_msg
PROTOBUF_NAMESPACE_OPEN
template<> ::image_msg::image* Arena::CreateMaybeMessage<::image_msg::image>(Arena*);
template<> ::image_msg::image_buf* Arena::CreateMaybeMessage<::image_msg::image_buf>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace image_msg {

// ===================================================================

class image PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:image_msg.image) */ {
 public:
  inline image() : image(nullptr) {};
  virtual ~image();

  image(const image& from);
  image(image&& from) noexcept
    : image() {
    *this = ::std::move(from);
  }

  inline image& operator=(const image& from) {
    CopyFrom(from);
    return *this;
  }
  inline image& operator=(image&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const image& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const image* internal_default_instance() {
    return reinterpret_cast<const image*>(
               &_image_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(image& a, image& b) {
    a.Swap(&b);
  }
  inline void Swap(image* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(image* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline image* New() const final {
    return CreateMaybeMessage<image>(nullptr);
  }

  image* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<image>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const image& from);
  void MergeFrom(const image& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(image* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "image_msg.image";
  }
  protected:
  explicit image(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_image_5fmsg_2eproto);
    return ::descriptor_table_image_5fmsg_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMatDataFieldNumber = 6,
    kTimeStampFieldNumber = 1,
    kHeightFieldNumber = 2,
    kWidthFieldNumber = 3,
    kChannelFieldNumber = 4,
    kSizeFieldNumber = 5,
  };
  // bytes mat_data = 6;
  void clear_mat_data();
  const std::string& mat_data() const;
  void set_mat_data(const std::string& value);
  void set_mat_data(std::string&& value);
  void set_mat_data(const char* value);
  void set_mat_data(const void* value, size_t size);
  std::string* mutable_mat_data();
  std::string* release_mat_data();
  void set_allocated_mat_data(std::string* mat_data);
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  std::string* unsafe_arena_release_mat_data();
  GOOGLE_PROTOBUF_RUNTIME_DEPRECATED("The unsafe_arena_ accessors for"
  "    string fields are deprecated and will be removed in a"
  "    future release.")
  void unsafe_arena_set_allocated_mat_data(
      std::string* mat_data);
  private:
  const std::string& _internal_mat_data() const;
  void _internal_set_mat_data(const std::string& value);
  std::string* _internal_mutable_mat_data();
  public:

  // double time_stamp = 1;
  void clear_time_stamp();
  double time_stamp() const;
  void set_time_stamp(double value);
  private:
  double _internal_time_stamp() const;
  void _internal_set_time_stamp(double value);
  public:

  // int32 height = 2;
  void clear_height();
  ::PROTOBUF_NAMESPACE_ID::int32 height() const;
  void set_height(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_height() const;
  void _internal_set_height(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // int32 width = 3;
  void clear_width();
  ::PROTOBUF_NAMESPACE_ID::int32 width() const;
  void set_width(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_width() const;
  void _internal_set_width(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // int32 channel = 4;
  void clear_channel();
  ::PROTOBUF_NAMESPACE_ID::int32 channel() const;
  void set_channel(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_channel() const;
  void _internal_set_channel(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // int32 size = 5;
  void clear_size();
  ::PROTOBUF_NAMESPACE_ID::int32 size() const;
  void set_size(::PROTOBUF_NAMESPACE_ID::int32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int32 _internal_size() const;
  void _internal_set_size(::PROTOBUF_NAMESPACE_ID::int32 value);
  public:

  // @@protoc_insertion_point(class_scope:image_msg.image)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr mat_data_;
  double time_stamp_;
  ::PROTOBUF_NAMESPACE_ID::int32 height_;
  ::PROTOBUF_NAMESPACE_ID::int32 width_;
  ::PROTOBUF_NAMESPACE_ID::int32 channel_;
  ::PROTOBUF_NAMESPACE_ID::int32 size_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_image_5fmsg_2eproto;
};
// -------------------------------------------------------------------

class image_buf PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:image_msg.image_buf) */ {
 public:
  inline image_buf() : image_buf(nullptr) {};
  virtual ~image_buf();

  image_buf(const image_buf& from);
  image_buf(image_buf&& from) noexcept
    : image_buf() {
    *this = ::std::move(from);
  }

  inline image_buf& operator=(const image_buf& from) {
    CopyFrom(from);
    return *this;
  }
  inline image_buf& operator=(image_buf&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const image_buf& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const image_buf* internal_default_instance() {
    return reinterpret_cast<const image_buf*>(
               &_image_buf_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(image_buf& a, image_buf& b) {
    a.Swap(&b);
  }
  inline void Swap(image_buf* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(image_buf* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline image_buf* New() const final {
    return CreateMaybeMessage<image_buf>(nullptr);
  }

  image_buf* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<image_buf>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const image_buf& from);
  void MergeFrom(const image_buf& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(image_buf* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "image_msg.image_buf";
  }
  protected:
  explicit image_buf(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_image_5fmsg_2eproto);
    return ::descriptor_table_image_5fmsg_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kImageFieldNumber = 1,
  };
  // repeated .image_msg.image image_ = 1;
  int image__size() const;
  private:
  int _internal_image__size() const;
  public:
  void clear_image_();
  ::image_msg::image* mutable_image_(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::image_msg::image >*
      mutable_image_();
  private:
  const ::image_msg::image& _internal_image_(int index) const;
  ::image_msg::image* _internal_add_image_();
  public:
  const ::image_msg::image& image_(int index) const;
  ::image_msg::image* add_image_();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::image_msg::image >&
      image_() const;

  // @@protoc_insertion_point(class_scope:image_msg.image_buf)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::image_msg::image > image__;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_image_5fmsg_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// image

// double time_stamp = 1;
inline void image::clear_time_stamp() {
  time_stamp_ = 0;
}
inline double image::_internal_time_stamp() const {
  return time_stamp_;
}
inline double image::time_stamp() const {
  // @@protoc_insertion_point(field_get:image_msg.image.time_stamp)
  return _internal_time_stamp();
}
inline void image::_internal_set_time_stamp(double value) {
  
  time_stamp_ = value;
}
inline void image::set_time_stamp(double value) {
  _internal_set_time_stamp(value);
  // @@protoc_insertion_point(field_set:image_msg.image.time_stamp)
}

// int32 height = 2;
inline void image::clear_height() {
  height_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::_internal_height() const {
  return height_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::height() const {
  // @@protoc_insertion_point(field_get:image_msg.image.height)
  return _internal_height();
}
inline void image::_internal_set_height(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  height_ = value;
}
inline void image::set_height(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_height(value);
  // @@protoc_insertion_point(field_set:image_msg.image.height)
}

// int32 width = 3;
inline void image::clear_width() {
  width_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::_internal_width() const {
  return width_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::width() const {
  // @@protoc_insertion_point(field_get:image_msg.image.width)
  return _internal_width();
}
inline void image::_internal_set_width(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  width_ = value;
}
inline void image::set_width(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_width(value);
  // @@protoc_insertion_point(field_set:image_msg.image.width)
}

// int32 channel = 4;
inline void image::clear_channel() {
  channel_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::_internal_channel() const {
  return channel_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::channel() const {
  // @@protoc_insertion_point(field_get:image_msg.image.channel)
  return _internal_channel();
}
inline void image::_internal_set_channel(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  channel_ = value;
}
inline void image::set_channel(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_channel(value);
  // @@protoc_insertion_point(field_set:image_msg.image.channel)
}

// int32 size = 5;
inline void image::clear_size() {
  size_ = 0;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::_internal_size() const {
  return size_;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 image::size() const {
  // @@protoc_insertion_point(field_get:image_msg.image.size)
  return _internal_size();
}
inline void image::_internal_set_size(::PROTOBUF_NAMESPACE_ID::int32 value) {
  
  size_ = value;
}
inline void image::set_size(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _internal_set_size(value);
  // @@protoc_insertion_point(field_set:image_msg.image.size)
}

// bytes mat_data = 6;
inline void image::clear_mat_data() {
  mat_data_.ClearToEmpty(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline const std::string& image::mat_data() const {
  // @@protoc_insertion_point(field_get:image_msg.image.mat_data)
  return _internal_mat_data();
}
inline void image::set_mat_data(const std::string& value) {
  _internal_set_mat_data(value);
  // @@protoc_insertion_point(field_set:image_msg.image.mat_data)
}
inline std::string* image::mutable_mat_data() {
  // @@protoc_insertion_point(field_mutable:image_msg.image.mat_data)
  return _internal_mutable_mat_data();
}
inline const std::string& image::_internal_mat_data() const {
  return mat_data_.Get();
}
inline void image::_internal_set_mat_data(const std::string& value) {
  
  mat_data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value, GetArena());
}
inline void image::set_mat_data(std::string&& value) {
  
  mat_data_.Set(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:image_msg.image.mat_data)
}
inline void image::set_mat_data(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  
  mat_data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArena());
  // @@protoc_insertion_point(field_set_char:image_msg.image.mat_data)
}
inline void image::set_mat_data(const void* value,
    size_t size) {
  
  mat_data_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:image_msg.image.mat_data)
}
inline std::string* image::_internal_mutable_mat_data() {
  
  return mat_data_.Mutable(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline std::string* image::release_mat_data() {
  // @@protoc_insertion_point(field_release:image_msg.image.mat_data)
  return mat_data_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void image::set_allocated_mat_data(std::string* mat_data) {
  if (mat_data != nullptr) {
    
  } else {
    
  }
  mat_data_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), mat_data,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:image_msg.image.mat_data)
}
inline std::string* image::unsafe_arena_release_mat_data() {
  // @@protoc_insertion_point(field_unsafe_arena_release:image_msg.image.mat_data)
  GOOGLE_DCHECK(GetArena() != nullptr);
  
  return mat_data_.UnsafeArenaRelease(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      GetArena());
}
inline void image::unsafe_arena_set_allocated_mat_data(
    std::string* mat_data) {
  GOOGLE_DCHECK(GetArena() != nullptr);
  if (mat_data != nullptr) {
    
  } else {
    
  }
  mat_data_.UnsafeArenaSetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      mat_data, GetArena());
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:image_msg.image.mat_data)
}

// -------------------------------------------------------------------

// image_buf

// repeated .image_msg.image image_ = 1;
inline int image_buf::_internal_image__size() const {
  return image__.size();
}
inline int image_buf::image__size() const {
  return _internal_image__size();
}
inline void image_buf::clear_image_() {
  image__.Clear();
}
inline ::image_msg::image* image_buf::mutable_image_(int index) {
  // @@protoc_insertion_point(field_mutable:image_msg.image_buf.image_)
  return image__.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::image_msg::image >*
image_buf::mutable_image_() {
  // @@protoc_insertion_point(field_mutable_list:image_msg.image_buf.image_)
  return &image__;
}
inline const ::image_msg::image& image_buf::_internal_image_(int index) const {
  return image__.Get(index);
}
inline const ::image_msg::image& image_buf::image_(int index) const {
  // @@protoc_insertion_point(field_get:image_msg.image_buf.image_)
  return _internal_image_(index);
}
inline ::image_msg::image* image_buf::_internal_add_image_() {
  return image__.Add();
}
inline ::image_msg::image* image_buf::add_image_() {
  // @@protoc_insertion_point(field_add:image_msg.image_buf.image_)
  return _internal_add_image_();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::image_msg::image >&
image_buf::image_() const {
  // @@protoc_insertion_point(field_list:image_msg.image_buf.image_)
  return image__;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace image_msg

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_image_5fmsg_2eproto
