#ifndef GL_TEXTURE_OBJECT
#define GL_TEXTURE_OBJECT

namespace GLDraw {

/** @ingroup GLDraw
 * @brief A GL texture object class.  Simplifies allocating and cleaning up
 * texture objects.  Usually you only need to interact with GLTexture[X]D
 * and the texture management is handled for you.
 */
class GLTextureObject
{
public:
	GLTextureObject();
	GLTextureObject(GLTextureObject&);
	~GLTextureObject();

	void generate();
	void cleanup();
	void bind(unsigned int target) const;
	void unbind(unsigned int target) const;
	bool isNull() const;

	void stealObject(GLTextureObject&);

private:
	unsigned int glName;
};

} //namespace GLDraw

#endif
