#include "listing4.h"

#include <iostream>
#include <cstring>
#include <string>

int listing4_1()
{
	printf("Hello World 4-1\nSize");
	using namespace std;
	int yams[3];
	yams[0] = 7;
	yams[1] = 8;
	yams[2] = 6;

	int yamcosts[3] = { 20, 30, 5 };

	cout << "Toutal yams = " << yams[0] + yams[1] + yams[2] << endl;
	cout << "The package with " << yams[1] << " yams costs " << yamcosts[1] << " cents per yam." << endl;

	//...

	cout << "\nSize of yams array = " << sizeof yams << " bytes." << endl;
	cout << "Size of one element = " << sizeof yams[0] << " bytes." << endl;

	return 0;
}

int listing4_2()
{
	printf("Hello World 4-2\n");
	using namespace std;

	const int SIZE = 15;
	char name1[SIZE];
	char name2[SIZE] = "C++owboy";

	cout << "Howdy! I'm " << name2 << "! What's your name?" << endl;
	cin >> name1;
	cout << "Well, " << name1 << ", your name has " << strlen(name1) << " letters and is stored" << endl;
	cout << "in an array of " << sizeof(name1) << " bytes." << endl;
	cout << "Your initial is " << name1[0] << "." << endl;
	name2[3] = '\0';
	cout << "Here are the first 3 characters of my name: ";
	cout << name2 << endl;

	return 0;
}

int listing4_3()
{
	printf("Hello World 4-3\n");
	using namespace std;

	const int AR_SIZE = 20;
	char name[AR_SIZE];
	char dessert[AR_SIZE];

	cout << "Enter your name:" << endl;
	cin >> name;
	cout << "Enter your favorite dessert:" << endl;
	cin >> dessert;
	cout << "I have some delicious " << dessert;
	cout << " for you, " << name << ".\n";

	return 0;
}

int listing4_4()
{
	printf("Hello World 4-4\n");
	using namespace std;

	const int AR_SIZE = 20;
	char name[AR_SIZE];
	char dessert[AR_SIZE];

	cout << "Enter your name:" << endl;
	cin.getline(name, AR_SIZE);
	cout << "Enter your favorite dessert:" << endl;
	cin.getline(dessert, AR_SIZE);
	cout << "I have some delicious " << dessert;
	cout << " for you, " << name << ".\n";

	return 0;
}

int listing4_5()
{
	printf("Hello World 4-5\n");
	using namespace std;

	const int AR_SIZE = 20;
	char name[AR_SIZE];
	char dessert[AR_SIZE];

	cout << "Enter your name:" << endl;
	cin.get(name, AR_SIZE).get();
	cout << "Enter your favorite dessert:" << endl;
	cin.get(dessert, AR_SIZE).get();
	cout << "I have some delicious " << dessert;
	cout << " for you, " << name << ".\n";

	return 0;
}

int listing4_7()
{
	printf("Hello World 4-7\n");
	using namespace std;

	char x1[20];
	char x2[20] = "jaguar";
	string str1;
	string str2 = "panther";

	cout << "Enter a kind of feline: ";
	cin >> x1;
	cout << "Enter another kind of feline: ";
	cin >> str1;
	cout << "Here are some felines:\n";
	cout << x1 << " " << x2 << " " << str1 << " " << str2 << endl;
	cout << "The third letter in " << x2 << " is " << x2[2] << endl;
	cout << "The third letter in " << str2 << " is " << str2[2] << endl;

	return 0;
}

int listing4_8()
{
	printf("Hello World 4-8\n");
	using namespace std;

	string s1 = "penguin";
	string s2, s3;

	cout << "You can assign one string object to another: s2 = s1\n";
	s2 = s1;
	cout << "s1: " << s1 << ", s2: " << s2 << endl;
	cout << "You can assign a C-style string to a string object.\n";
	cout << "s2 = \"buzzard\"\n";
	s2 = "buzzard";
	cout << "s2: " << s2 << endl;
	cout << "You can cancatenate strings: s3 = s1 + s2\n";
	s3 = s1 + s2;
	cout << "s3: " << s3 << endl;
	cout << "You can append strings.\n";
	s1 += s2;
	cout << "s1 += s2 yields s1 = " << s1 << endl;
	s2 += " for a day";
	cout << "s2 += \" for a day\" yields s2 = " << s2 << endl;

	return 0;
}

int listing4_9()
{
	printf("Hello World 4-9\n");
	using namespace std;

	char x1[20];
	char x2[20] = "jaguar";
	string str1;
	string str2 = "panther";

	// assignment
	str1 = str2;
	strcpy_s(x1, x2);

	// appending
	str1 += " paste";
	strcat_s(x1, " juice");

	// finding the length
	int len1 = str1.size();
	int len2 = strlen(x1);

	cout << "The string " << str1 << " contains " << len1 << " characters.\n";
	cout << "The string " << x1 << " contains " << len2 << " characters.\n";

	return 0;
}

int listing4_10()
{
	printf("Hello World 4-10\n");
	using namespace std;

	char s[20];
	string str;

	cout << strlen(s) << endl;
	cout << str.size() << endl;
	cout << "Enter a line of text:\n";
	cin.getline(s, 20);
	getline(cin, str);
	cout << strlen(s) << endl;
	cout << str.size() << endl;

	return 0;
}

int listing4_11()
{
	printf("Hello World 4-11\n");
	using namespace std;

	struct inflatable
	{
		char name[20];
		float volume;
		double price;
	};

	inflatable guest = {
		"Glorious Gloria",
		1.88,
		29.99
	};
	inflatable pal = {
		"Audacious Arthur",
		3.12,
		32.99
	};

	cout << guest.name << endl;
	cout << pal.name << endl;
	cout << guest.price + pal.price << endl;

	return 0;
}

int listing4_12()
{
	printf("Hello World 4-12\n");
	using namespace std;

	struct inflatable
	{
		char name[20];
		float volume;
		double price;
	};

	inflatable bouquet = {
		"sunflowers",
		0.20,
		12.49
	};
	inflatable choice;
	cout << "bouquet: " << bouquet.name << " for $";
	cout << bouquet.price << endl;
	choice = bouquet; // assign one structure to another
	cout << "choice: " << choice.name << " for $";
	cout << choice.price << endl;

	return 0;
}

int listing4_13()
{
	printf("Hello World 4-13\n");
	using namespace std;

	struct inflatable
	{
		char name[20];
		float volume;
		double price;
	};

	inflatable guests[2] = // initializing an array of structs
	{
		{"Bambi", 0.5, 21.99}, // first structure in array
		{"Godzilla", 2000, 565.99} // next structure in array
	};
	cout << "The guests " << guests[0].name << " and " << guests[1].name
		<< "\nhave a combined volume of "
		<< guests[0].volume + guests[1].volume << " cubic feet.\n";

	return 0;
}