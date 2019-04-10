#pragma once
#ifndef JL2CPP_HPP
#define JL2CPP_HPP

#include<cstdlib>
#include<cstring>
#include<cmath>
#include<cstdio>
#include<string>
#include<type_traits>
#include<functional>
#include<random>
#include "Eigen/Dense"
#include "unsupported/Eigen/SpecialFunctions"
namespace jl2cpp{
using Eigen::pow;
struct matrix {
	void *ptr;
	int64_t rsize;
	int64_t csize;
};
struct tuple {
	void* ptr;
	std::size_t size;
};

// get sum of size of each elements of a tuple
template<typename T, size_t idx>struct SumTupleSizeImpl{
    static const size_t value =  sizeof(typename std::tuple_element<idx, T>::type) + SumTupleSizeImpl<T, idx - 1>::value;
};
template<typename T>struct SumTupleSizeImpl<T, 0>{
    static const size_t value = sizeof(typename std::tuple_element<0, T>::type);
};
template<typename T>struct SumTupleSize{
    static const size_t value = SumTupleSizeImpl<T, std::tuple_size<T>::value - 1>::value;
};


// tweaks to apply specified member functions if available
// cf. https://cpplover.blogspot.jp/2010/03/decltypesfinae.html

// apply eval() if the argument has eval() as member function
template<typename T> T&& value();
template<typename T> struct has_eval {
	template<typename U> static auto check(U v)->decltype(v.eval(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};
template<typename T> auto eval_if_possible(T v)->decltype(v.eval()) {
	return v.eval();
}	
template<typename T, typename = typename std::enable_if<!has_eval<T>::value>::type>
T EIGEN_STRONG_INLINE eval_if_possible(T const v) {
	return v;
}

// apply data() and wrap if the argument is an array
template<typename T> struct has_data {
	template<typename U> static auto check(U v)->decltype(v.data(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};
template<typename T, typename = typename std::enable_if<has_data<T>::value>::type>
EIGEN_STRONG_INLINE matrix wrap_array(T const& v) {
	printf("%lld*%lld=%lld\n", v.rows(), v.cols(), v.size());
	void* ret = malloc(sizeof(typename T::value_type) * v.size());
	if(ret == nullptr){printf("rack of memory\n");}
	printf("malloced\n");
	printf("first: %f\n", v(0));
	memmove((void*)ret, v.data(), sizeof(typename T::value_type) * v.size());
	return matrix{ret, v.rows(), v.cols()};
	//return matrix{(void*)v.data(), v.rows(), v.cols()};
}
template<typename T, typename = typename std::enable_if<!has_data<T>::value>::type>
EIGEN_STRONG_INLINE T wrap_array(T const& v) {
	return v;
}

template<typename T, typename = typename std::enable_if<has_data<T>::value>::type>
EIGEN_STRONG_INLINE auto get_one_elem(T const& v)->decltype(*(v.data())){
	return *(v.data());
}
template<typename T, typename = typename std::enable_if<!has_data<T>::value>::type>
EIGEN_STRONG_INLINE T get_one_elem(T const& v) {
	return v;
}

// for returning array
template<typename T> EIGEN_STRONG_INLINE auto calc_retval(T const& v)->decltype(wrap_array(eval_if_possible(v))) { // C++11 compliant
	return wrap_array(eval_if_possible(v));
}

/*
template<typename T> struct can_get {
	template<typename U> static auto check(U v)->decltype(std::get<0>(v), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};

template<typename T, typename = typename std::enable_if<can_get<T>::value>::type>
EIGEN_STRONG_INLINE tuple wrap_tuple(T const& v) {
	void* ret = malloc(SumTupleSize<T>::value);
	std::size_t offset = 0;
	for(int i = 0; i < std::tuple_size<T>::value; i++) {
		(decltype *)(ret + offset)
	}
	return matrix{ret, SumTupleSize<T>::value};
}
template<typename T, typename = typename std::enable_if<!can_get<T>::value>::type>
EIGEN_STRONG_INLINE T wrap_tuple(T const& v) {
	return v;
}
*/

template<typename T> struct has_array {
	template<typename U> static auto check(U v)->decltype(v.array(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};

template<typename T> EIGEN_STRONG_INLINE auto array_if_possible(T const& v)->decltype(v.array()) {
	return v.array();
}	
template<typename T, typename = typename std::enable_if<!has_array<T>::value>::type>
T EIGEN_STRONG_INLINE array_if_possible(T const& v) {
	return v;
}

template<typename T> struct has_matrix {
	template<typename U> static auto check(U v)->decltype(v.matrix(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};

template<typename T> EIGEN_STRONG_INLINE auto matrix_if_possible(T const& v)->decltype(v.matrix()) {
	return v.matrix();
}	
template<typename T, typename = typename std::enable_if<!has_matrix<T>::value>::type>
T EIGEN_STRONG_INLINE matrix_if_possible(T const& v) {
	return v;
}


template<typename T> struct has_size {
	template<typename U> static auto check(U v)->decltype(v.size(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};

template<typename T> EIGEN_STRONG_INLINE auto get_size(T const& v)->decltype(v.size()) {
	return v.size();
}	
template<typename T, typename = typename std::enable_if<!has_array<T>::value>::type>
int EIGEN_STRONG_INLINE get_size(T const& v) {
	return 1;
}

template<typename T> struct has_cols {
	template<typename U> static auto check(U v)->decltype(v.cols(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};

template<typename T> EIGEN_STRONG_INLINE auto get_cols(T const& v)->decltype(v.cols()) {
	return v.cols();
}	
template<typename T, typename = typename std::enable_if<!has_array<T>::value>::type>
int EIGEN_STRONG_INLINE get_cols(T const& v) {
	return 1;
}

template<typename T> struct has_rows {
	template<typename U> static auto check(U v)->decltype(v.rows(), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};

template<typename T> EIGEN_STRONG_INLINE auto get_rows(T const& v)->decltype(v.rows()) {
	return v.rows();
}
template<typename T, typename = typename std::enable_if<!has_array<T>::value>::type>
int EIGEN_STRONG_INLINE get_rows(T const& v) {
	return 1;
}
/*
template<typename T> struct has_Scalar {
	template<typename U> static auto check(U v)->decltype(sizeof(typename T::Scalar), std::true_type());
	static std::false_type check(...);
	static bool const value = decltype(check(jl2cpp::value<T>()))::value;
};
template<typename T> EIGEN_STRONG_INLINE auto get_rows(T const& v)->decltype(v.rows()) {
	return v.rows();
}
template<typename T, typename = typename std::enable_if<!has_array<T>::value>::type>
int EIGEN_STRONG_INLINE get_rows(T const& v) {
	return 1;
}
*/

#define JL2CPP_CWISE_OPERATOR(name, op)\
template<typename T, typename U>\
EIGEN_STRONG_INLINE auto name(T const& l, U const& r)->decltype(matrix_if_possible(array_if_possible(l) op array_if_possible(r))) {\
	return matrix_if_possible(array_if_possible(l) op array_if_possible(r));\
}

#define JL2CPP_CWISE_COMPARE_OPERATOR(name, op)\
template<typename T, typename U>\
EIGEN_STRONG_INLINE auto name(T const& l, U const& r)->decltype(matrix_if_possible(array_if_possible(l) op array_if_possible(r)).template cast<int64_t>()) {\
	return matrix_if_possible(array_if_possible(l) op array_if_possible(r)).template cast<int64_t>();\
}

#define JL2CPP_CWISE_FUNCTION(name, fun)\
template<typename T, typename U>\
EIGEN_STRONG_INLINE auto name(T const& l, U const& r)->decltype(matrix_if_possible(fun(array_if_possible(l), array_if_possible(r)))) {\
	return matrix_if_possible(fun(array_if_possible(l), array_if_possible(r)));\
}\

#define JL2CPP_STD_UNARY_FUN(name)\
template<typename T, typename = typename std::enable_if<jl2cpp::has_array<T>::value>::type>\
EIGEN_STRONG_INLINE auto name(T const& v)->decltype(matrix_if_possible(Eigen::name(array_if_possible(v)))) {\
	/*return matrix_if_possible(Eigen::name(array_if_possible(v)));*/\
	return matrix_if_possible(array_if_possible(v).name());\
}\
template<typename T, typename = typename std::enable_if<!jl2cpp::has_array<T>::value>::type>\
EIGEN_STRONG_INLINE auto name(T const& v)->decltype(std:: name(v)) {\
	return std:: name (v);\
}\

JL2CPP_CWISE_OPERATOR(add, +)
JL2CPP_CWISE_OPERATOR(sub, -)
JL2CPP_CWISE_OPERATOR(cwiseProduct, *)
JL2CPP_CWISE_OPERATOR(cwiseQuotient, /)
JL2CPP_CWISE_COMPARE_OPERATOR(cwiseLt, <)
JL2CPP_CWISE_COMPARE_OPERATOR(cwiseLte, <=)
JL2CPP_CWISE_COMPARE_OPERATOR(cwiseGt, >)
JL2CPP_CWISE_COMPARE_OPERATOR(cwiseGte, >=)
JL2CPP_CWISE_COMPARE_OPERATOR(cwiseEqual, ==)
JL2CPP_CWISE_COMPARE_OPERATOR(cwiseNotEqual, !=)

// unary eigen functions which same names exist in std
JL2CPP_STD_UNARY_FUN(abs)
JL2CPP_STD_UNARY_FUN(sqrt)
JL2CPP_STD_UNARY_FUN(log)
JL2CPP_STD_UNARY_FUN(log10)
JL2CPP_STD_UNARY_FUN(exp)
JL2CPP_STD_UNARY_FUN(sin)
JL2CPP_STD_UNARY_FUN(cos)
JL2CPP_STD_UNARY_FUN(tan)
JL2CPP_STD_UNARY_FUN(asin)
JL2CPP_STD_UNARY_FUN(acos)
JL2CPP_STD_UNARY_FUN(atan)
JL2CPP_STD_UNARY_FUN(sinh)
JL2CPP_STD_UNARY_FUN(cosh)
JL2CPP_STD_UNARY_FUN(tanh)
JL2CPP_STD_UNARY_FUN(floor)
JL2CPP_STD_UNARY_FUN(ceil)
JL2CPP_STD_UNARY_FUN(round)
JL2CPP_STD_UNARY_FUN(isnan)
JL2CPP_STD_UNARY_FUN(isfinite)
JL2CPP_STD_UNARY_FUN(isinf)
// JL2CPP_STD_UNARY_FUN(erf) // Eigen's implementation is not working, idk why
JL2CPP_STD_UNARY_FUN(erfc)

// we can't create lambda in decltype
EIGEN_STRONG_INLINE double elfImpl(double x){return std::erf(x);}

//cwise erf
template<typename T, typename = typename std::enable_if<has_array<T>::value>::type>
EIGEN_STRONG_INLINE auto erf(T const& v)->decltype(matrix_if_possible(array_if_possible(v).unaryExpr(std::ptr_fun(elfImpl)))) {
	/*return matrix_if_possible(Eigen::name(array_if_possible(v)));*/
	return matrix_if_possible(array_if_possible(v).unaryExpr(std::ptr_fun(elfImpl)));
}
template<typename T, typename = typename std::enable_if<!has_array<T>::value>::type>
EIGEN_STRONG_INLINE auto erf(T const& v)->decltype(std::erf(v)) {
	return std::erf(v);
}


// cwisePow
// the case of using Eigen's pow
template<typename T, typename U, typename = typename std::enable_if<has_array<T>::value || has_array<U>::value>::type>
EIGEN_STRONG_INLINE auto cwisePow(T const& l, U const& r)->decltype(matrix_if_possible(Eigen::pow(array_if_possible(l), array_if_possible(r)))) {
	return matrix_if_possible(Eigen::pow(array_if_possible(l), array_if_possible(r)));
}
//the case of using std's pow
template<typename T, typename U, typename = typename std::enable_if<!has_array<T>::value && !has_array<U>::value>::type>
EIGEN_STRONG_INLINE auto cwisePow(T const& l, U const& r)->decltype(std::pow(l, r)) {
	return std::pow(l, r);
}

// pow
template<typename T, typename U, typename = typename std::enable_if<has_array<T>::value && !has_array<U>::value>::type>
EIGEN_STRONG_INLINE auto pow(T const& l, U const& r)->decltype(l.eval()) {
	auto ret = l.eval();
	for(int i = 0; i < eval_if_possible(r) - 1; i++) {
		ret *= l;
	}
	return ret;
}
template<typename T, typename U, typename = typename std::enable_if<!has_array<T>::value && !has_array<U>::value>::type>
EIGEN_STRONG_INLINE auto pow(T const& l, U const& r)->decltype(std::pow(l, r)) {
	return std::pow(l, r);
}

template<typename T>
EIGEN_STRONG_INLINE auto random(T const& t)->decltype((((Eigen::VectorXd::Random(t)).array() + 1.0) / 2.0).matrix()) {
	return (((Eigen::VectorXd::Random(t)).array() + 1.0) / 2.0).matrix();
}
template<typename T, typename U>
EIGEN_STRONG_INLINE auto random(T const& t, U const& u)->decltype((((Eigen::MatrixXd::Random(t, u)).array() + 1.0) / 2.0).matrix()) {
	return (((Eigen::MatrixXd::Random(t, u)).array() + 1.0) / 2.0).matrix();
}
template<typename T, typename U>
EIGEN_STRONG_INLINE T arith_rshift(T const l, U const r){
	return (l >= 0? (l >> r): (((-1 * l) >> r) * -1));
}
template<typename T, typename U>
EIGEN_STRONG_INLINE T logic_rshift(T const l, U const r){
	if(l >= 0) {
		return l >> r;
	}else{
		return static_cast<T>(static_cast<typename std::make_unsigned<T>::type>(l) >> r);
	}
}

template<typename T, typename U = double>
EIGEN_STRONG_INLINE Eigen::Matrix<U, Eigen::Dynamic, Eigen::Dynamic> randn(T const r, T const c) {
	std::random_device rd;
	std::default_random_engine engine(rd());
	std::normal_distribution<U> dist(0.0, 1.0);
	Eigen::Matrix<U, Eigen::Dynamic, Eigen::Dynamic> ret(r, c);
	auto ret2 = ret << dist(engine);
	for(T i = 0; i < r * c - 1; i++) {
		ret2, dist(engine);
	}
	ret2.finished();
	return ret;
}

template<typename T>
EIGEN_STRONG_INLINE double stdDev(T const& e) {
	auto v = e.array();
	return std::sqrt((v - v.mean()).square().sum() / (v.size() - 1.0));
}

template<typename T>
EIGEN_STRONG_INLINE std::string hex(T const n) {
	// the largest bit width we treat is 64, so the length of reslt should less than (or equal to) 16+1
	char buf[32];
	std::snprintf(buf, 32, "%x", n); // we use snprintf instead of stringstream for speed
	return std::string(buf);
}

template<typename T>
EIGEN_STRONG_INLINE T parse(std::string const& s, int base) {
	return static_cast<T>(std::strtol(s.c_str(), nullptr, base));
}

// type casts

// return int64_t if T is int32_t 
template<typename T> struct expand_if_int32 {
	typedef typename std::conditional<std::is_same<T, int32_t>::value, int64_t, T>::type type;
};
template<typename T>
struct ElType{
	using type = typename expand_if_int32<typename std::remove_const<typename std::remove_reference<T>::type>::type>::type;
};

// util



}
#endif