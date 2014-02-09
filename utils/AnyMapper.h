#ifndef ANY_MAPPER_H
#define ANY_MAPPER_H

#include "AnyCollection.h"

/** @brief An any-schema is a simple validation and mapping scheme for
 * AnyCollections.  The keys in the document must match up with the schema.
 *
 * AnyCollection schema;
 * schema["foo"]=_0;
 * schema["bar"]=5;
 * AnyCollection item;
 * item["foo"]=4;
 * item["bar"]=5;
 * 
 * AnyCollection matches;
 * Match(item,schema) [returns true]
 * Match(item,schema,matches) [returns true, sets matches to the map {0:4}]
 *
 * They can also be used to map matched items to new schemas
 * AnyCollection outschema,result;
 * outschema["foo2"]=_0;
 * outschema["bar2"]=10;
 * Fill(outschema,matches,result) [returns true, fills result with
 *    {"foo2":4,"bar2":10}]
 *
 * This can also be done in a single step:
 * MatchAndFill(item,schema,outschema,result);
 */

/** @brief A placeholder in an any-schema. The special value -1 matches
 * everything.
 */
class Wildcard
{
 public:
  Wildcard(int _id) : id(_id) {}
  int id;
};
const static Wildcard _Any(-1);
const static Wildcard _0(0);
const static Wildcard _1(1);
const static Wildcard _2(2);
const static Wildcard _3(3);
const static Wildcard _4(4);
const static Wildcard _5(5);
const static Wildcard _6(6);
const static Wildcard _7(7);
const static Wildcard _8(8);
const static Wildcard _9(9);
const static Wildcard _10(10);
inline std::ostream& operator << (std::ostream& out,const Wildcard& w) { out<< w.id; return out; }

bool Match(const AnyCollection& item,const AnyCollection& schema);
bool Match(const AnyCollection& item,const AnyCollection& schema,AnyCollection& wildcardValues);
bool Fill(const AnyCollection& schema,const AnyCollection& wildcardValues,AnyCollection& out);
bool MatchAndFill(const AnyCollection& in,const AnyCollection& inschema,const AnyCollection& outschema,AnyCollection& out);

#endif
