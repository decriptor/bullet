
typedef unsigned char BYTE;

GLuint LoadTextureRAW( const char * filename, int wrap , int width, int height )
{
	GLuint texture;
	BYTE * data;
	FILE * file;

	// open texture data
	file = fopen( filename, "rb" );
	if ( file == NULL ) return 0;

	// allocate buffer
	//width = 1024;
	//height = 1024;
	data = malloc( width * height * 3 +10);

	// read texture data
	fread( data, width * height * 3, 1, file );
	fclose( file );

	// allocate a texture name
	glGenTextures( 1, &texture );

	// select our current texture
	glBindTexture( GL_TEXTURE_2D, texture );

	// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );

	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );

	// when texture area is large, bilinear filter the first mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// if wrap is true, the texture wraps over at the edges (repeat)
	//       ... false, the texture ends at the edges (clamp)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
			wrap ? GL_REPEAT : GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
			wrap ? GL_REPEAT : GL_CLAMP );

	// build our texture mipmaps
	glTexImage2D(GL_TEXTURE_2D,             // Load a 2D image
			0,
			GL_RGB,
			width,        height,
			0,
			GL_RGB,
			GL_UNSIGNED_BYTE,
			data);
	// free buffer
	free( data );

	return texture;
}
