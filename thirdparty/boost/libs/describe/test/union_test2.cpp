// Copyright 2020, 2021 Peter Dimov
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt

#include <boost/describe/members.hpp>
#include <boost/describe/class.hpp>
#include <boost/core/lightweight_test.hpp>
#include <boost/config/pragma_message.hpp>

#if !defined(BOOST_DESCRIBE_CXX11)

BOOST_PRAGMA_MESSAGE("Skipping test because C++11 is not available")
int main() {}

#elif defined(__GNUC__) && __GNUC__ < 5

BOOST_PRAGMA_MESSAGE("Skipping test because g++ 4.8")
int main() {}

#else

union A
{
public:

    int m1;
    static int m2;
    int f1() const { return m1; }
    static int f2() { return m2; }

protected:

    int m3;
    static int m4;
    int f3() const { return m3; }
    static int f4() { return m4; }

private:

    int m5;
    static int m6;
    int f5() const { return m5; }
    static int f6() { return m6; }

    BOOST_DESCRIBE_CLASS(A, (), (m1, m2, f1, f2), (m3, m4, f3, f4), (m5, m6, f5, f6))

    friend int main();
};

int A::m2;
int A::m4;
int A::m6;

#if !defined(BOOST_DESCRIBE_CXX14)

BOOST_PRAGMA_MESSAGE("Skipping test because C++14 is not available")
int main() {}

#else

#include <boost/mp11.hpp>

int main()
{
    using namespace boost::describe;
    using namespace boost::mp11;

    {
        using L = describe_members<A, mod_public | mod_any_member>;

        BOOST_TEST_EQ( mp_size<L>::value, 4 );

        using D1 = mp_at_c<L, 0>;
        using D2 = mp_at_c<L, 1>;
        using D3 = mp_at_c<L, 2>;
        using D4 = mp_at_c<L, 3>;

        BOOST_TEST( D1::pointer == &A::m1 );
        BOOST_TEST_CSTR_EQ( D1::name, "m1" );
        BOOST_TEST_EQ( D1::modifiers, mod_public );

        BOOST_TEST( D2::pointer == &A::m2 );
        BOOST_TEST_CSTR_EQ( D2::name, "m2" );
        BOOST_TEST_EQ( D2::modifiers, mod_public | mod_static );

        BOOST_TEST( D3::pointer == &A::f1 );
        BOOST_TEST_CSTR_EQ( D3::name, "f1" );
        BOOST_TEST_EQ( D3::modifiers, mod_public | mod_function );

        BOOST_TEST( D4::pointer == &A::f2 );
        BOOST_TEST_CSTR_EQ( D4::name, "f2" );
        BOOST_TEST_EQ( D4::modifiers, mod_public | mod_static | mod_function );
    }

    {
        using L = describe_members<A, mod_protected | mod_any_member>;

        BOOST_TEST_EQ( mp_size<L>::value, 4 );

        using D1 = mp_at_c<L, 0>;
        using D2 = mp_at_c<L, 1>;
        using D3 = mp_at_c<L, 2>;
        using D4 = mp_at_c<L, 3>;

        BOOST_TEST( D1::pointer == &A::m3 );
        BOOST_TEST_CSTR_EQ( D1::name, "m3" );
        BOOST_TEST_EQ( D1::modifiers, mod_protected );

        BOOST_TEST( D2::pointer == &A::m4 );
        BOOST_TEST_CSTR_EQ( D2::name, "m4" );
        BOOST_TEST_EQ( D2::modifiers, mod_protected | mod_static );

        BOOST_TEST( D3::pointer == &A::f3 );
        BOOST_TEST_CSTR_EQ( D3::name, "f3" );
        BOOST_TEST_EQ( D3::modifiers, mod_protected | mod_function );

        BOOST_TEST( D4::pointer == &A::f4 );
        BOOST_TEST_CSTR_EQ( D4::name, "f4" );
        BOOST_TEST_EQ( D4::modifiers, mod_protected | mod_static | mod_function );
    }

    {
        using L = describe_members<A, mod_private | mod_any_member>;

        BOOST_TEST_EQ( mp_size<L>::value, 4 );

        using D1 = mp_at_c<L, 0>;
        using D2 = mp_at_c<L, 1>;
        using D3 = mp_at_c<L, 2>;
        using D4 = mp_at_c<L, 3>;

        BOOST_TEST( D1::pointer == &A::m5 );
        BOOST_TEST_CSTR_EQ( D1::name, "m5" );
        BOOST_TEST_EQ( D1::modifiers, mod_private );

        BOOST_TEST( D2::pointer == &A::m6 );
        BOOST_TEST_CSTR_EQ( D2::name, "m6" );
        BOOST_TEST_EQ( D2::modifiers, mod_private | mod_static );

        BOOST_TEST( D3::pointer == &A::f5 );
        BOOST_TEST_CSTR_EQ( D3::name, "f5" );
        BOOST_TEST_EQ( D3::modifiers, mod_private | mod_function );

        BOOST_TEST( D4::pointer == &A::f6 );
        BOOST_TEST_CSTR_EQ( D4::name, "f6" );
        BOOST_TEST_EQ( D4::modifiers, mod_private | mod_static | mod_function );
    }

    return boost::report_errors();
}

#endif // !defined(BOOST_DESCRIBE_CXX14)
#endif // !defined(BOOST_DESCRIBE_CXX11)
