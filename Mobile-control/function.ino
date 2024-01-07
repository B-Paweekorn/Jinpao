float strToFloat(String str) {
  float result = 0.0;
  float decimalMultiplier = 1.0;
  bool isNegative = false;
  bool decimalPart = false;
  int length = str.length();

  if (str[0] == '-') {
    isNegative = true;
    str = str.substring(1);
    length--;
  }

  for (int i = 0; i < length; i++) {
    char c = str[i];

    if (c == '.') {
      decimalPart = true;
      continue;
    }

    if (decimalPart) {
      decimalMultiplier *= 0.1;
      result += (c - '0') * decimalMultiplier;
    } else {
      result = result * 10.0 + (c - '0');
    }
  }

  if (isNegative) {
    result = -result;
  }

  return result;
}