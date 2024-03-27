use core::ops::Deref;
use defmt::Format;
use packed_struct::PackedStruct;

/// wrapper type because packed_struct doesn't implement float
/// packing by default
#[derive(Debug, Format, Clone, Default, Copy)]
pub struct PackedFloat(f32);

pub trait ToRegularArray<const T: usize> {
    fn to_regular_array(&self) -> &[f32; T];
}

impl<const T: usize> ToRegularArray<T> for [PackedFloat; T] {
    fn to_regular_array(&self) -> &[f32; T] {
        unsafe { &*(self as *const _ as *const _) }
    }
}

pub trait ToPackedFloatArray<const T: usize> {
    fn to_packed_float_array(&self) -> &[PackedFloat; T];
}

impl<const T: usize> ToPackedFloatArray<T> for [f32; T] {
    fn to_packed_float_array(&self) -> &[PackedFloat; T] {
        unsafe { &*(self as *const _ as *const _) }
    }
}

impl PackedStruct for PackedFloat {
    type ByteArray = [u8; 4];

    fn pack(&self) -> packed_struct::PackingResult<Self::ByteArray> {
        Ok(self.to_be_bytes())
    }

    fn unpack(src: &Self::ByteArray) -> packed_struct::PackingResult<Self> {
        Ok(Self(f32::from_be_bytes(*src)))
    }
}

impl Deref for PackedFloat {
    type Target = f32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
