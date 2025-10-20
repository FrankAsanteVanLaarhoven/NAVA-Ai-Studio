// Test file to verify Monaco Editor web worker configuration
console.log('Testing Monaco Editor configuration...');

// Simple function to test syntax highlighting
function testFunction() {
  const message = "Monaco Editor Test";
  console.log(message);
  return message;
}

// Test NAVΛ syntax
const navTest = `
// Navigation test
nav test_path(start: State, goal: State) -> Navigation {
  let distance = ∇⋋(start, goal)
  let path = 𝒩ℐ(distance)
  
  compose(
    tensor(start ⊗⋋ path),
    evolution(ℰ(path)),
    feedback(goal)
  )
}
`;

console.log('NAVΛ Test:', navTest);