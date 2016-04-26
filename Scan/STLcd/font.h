const unsigned char font[96][7] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x30, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x28, 0x7c, 0x28, 0x7c, 0x28, 0x00, 0x00, }, 
	{ 0x74, 0x54, 0xfe, 0x54, 0x5c, 0x00, 0x00, }, 
	{ 0x38, 0x28, 0x38, 0x00, 0x3e, 0x00, 0x0e, }, 
	{ 0x1e, 0x72, 0x5a, 0x74, 0x02, 0x00, 0x00, }, 
	{ 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x81, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x20, 0x38, 0x70, 0x38, 0x20, 0x00, 0x00, }, 
	{ 0x10, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x0e, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x22, 0x3e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x10, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x26, 0x2a, 0x32, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x22, 0x2a, 0x3e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x38, 0x08, 0x3e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x3a, 0x2a, 0x2c, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x2a, 0x2e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x20, 0x20, 0x3e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x2a, 0x3e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x38, 0x28, 0x3e, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x04, 0x0a, 0x0a, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x0a, 0x0a, 0x04, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x40, 0x5a, 0x70, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x42, 0x5a, 0x5a, 0x4a, 0x7a, 0x00, }, 
	{ 0x7e, 0x50, 0x50, 0x7e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x52, 0x52, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x42, 0x42, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x52, 0x52, 0x42, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x50, 0x50, 0x40, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x42, 0x52, 0x5e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x10, 0x10, 0x7e, 0x00, 0x00, 0x00, }, 
	{ 0x42, 0x42, 0x7e, 0x42, 0x42, 0x00, 0x00, }, 
	{ 0x42, 0x42, 0x7e, 0x40, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x10, 0x70, 0x1e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x40, 0x60, 0x40, 0x7e, 0x00, 0x00, }, 
	{ 0x7e, 0x20, 0x10, 0x7e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x42, 0x42, 0x7e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x50, 0x50, 0x70, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x42, 0x43, 0x7e, 0x00, 0x00, 0x00, }, 
	{ 0x7e, 0x50, 0x5e, 0x70, 0x00, 0x00, 0x00, }, 
	{ 0x72, 0x52, 0x52, 0x5e, 0x00, 0x00, 0x00, }, 
	{ 0x40, 0x40, 0x7e, 0x40, 0x40, 0x00, 0x00, }, 
	{ 0x7e, 0x02, 0x02, 0x7e, 0x00, 0x00, 0x00, }, 
	{ 0x78, 0x04, 0x02, 0x04, 0x78, 0x00, 0x00, }, 
	{ 0x7e, 0x02, 0x06, 0x02, 0x7e, 0x00, 0x00, }, 
	{ 0x42, 0x24, 0x18, 0x24, 0x42, 0x00, 0x00, }, 
	{ 0x70, 0x10, 0x1e, 0x10, 0x70, 0x00, 0x00, }, 
	{ 0x46, 0x4a, 0x52, 0x62, 0x00, 0x00, 0x00, }, 
	{ 0xff, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x70, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x81, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x20, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x00, }, 
	{ 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x28, 0x28, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x2a, 0x2a, 0x1e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x22, 0x22, 0x1e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x2a, 0x2a, 0x22, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x28, 0x28, 0x20, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x22, 0x2a, 0x2e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x08, 0x08, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x22, 0x22, 0x3e, 0x22, 0x22, 0x00, 0x00, }, 
	{ 0x22, 0x22, 0x3e, 0x20, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x08, 0x38, 0x0e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x20, 0x30, 0x20, 0x3e, 0x00, 0x00, }, 
	{ 0x3e, 0x10, 0x08, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x22, 0x22, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x28, 0x28, 0x38, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x22, 0x23, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x3e, 0x28, 0x2e, 0x38, 0x00, 0x00, 0x00, }, 
	{ 0x3a, 0x2a, 0x2a, 0x2e, 0x00, 0x00, 0x00, }, 
	{ 0x20, 0x20, 0x3e, 0x20, 0x20, 0x00, 0x00, }, 
	{ 0x3e, 0x02, 0x02, 0x3e, 0x00, 0x00, 0x00, }, 
	{ 0x38, 0x04, 0x02, 0x04, 0x38, 0x00, 0x00, }, 
	{ 0x3e, 0x02, 0x06, 0x02, 0x3e, 0x00, 0x00, }, 
	{ 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 0x00, }, 
	{ 0x38, 0x08, 0x0e, 0x08, 0x38, 0x00, 0x00, }, 
	{ 0x26, 0x2a, 0x32, 0x22, 0x00, 0x00, 0x00, }, 
	{ 0x10, 0xef, 0x81, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x81, 0xef, 0x10, 0x00, 0x00, 0x00, 0x00, }, 
	{ 0x18, 0x10, 0x08, 0x18, 0x00, 0x00, 0x00, }, 
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }, 
};